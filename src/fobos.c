/* Written by KC2DAC Dec 2024, adapted from existing KA9Q SDR handler programs
   Modified by Phil Karn KA9Q Feb 2024: approximate gain scaling, got direct sample mode working
   Modified by Phil Karn May 2026: handle repeated stop/start
In direct sample mode the two A/Ds are fed directly by the two HF inputs through fixed 20 dB gain
amplifiers, so there is no manual or automatic gain control.

In tuner mode "agc = yes" protects the A/D from overload (fobos_agc below): the gain
comes down fast when samples reach full scale or the level is too high, and goes
back up slowly, never above agc-max-gain (default: the configured lna_gain and
vga_gain). A/D samples at full scale are counted in either mode.

The FFT is currently complex in both tuner and direct sample mode. In tuner mode it accepts a complex sample
stream centered on the tuner LO.

In direct sample mode you can also treat the inputs as complex, or select or combine them
on a per-channel basis by selecting output filter type BEAM and calling set_filter_weights() in filter.c
*/

// enable new raw-mode upcall added to library
// Requires my modified libfobos library with raw mode support
#define RAW 1
#define FOBOS_NONTEMPORAL_STORES 1

#include <assert.h>
#include <errno.h>
#include <fobos.h>
#include <iniparser/iniparser.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <sysexits.h>
#include <stdatomic.h>

#include "config.h"
#include "misc.h"
#include "radio.h"
#include "sched.h"
#include "timing.h"
#include "defaults.h"
#include <time.h>

static float Power_alpha; // Exponential smoothing parameter for power estimation
static atomic_flag Name_set = ATOMIC_FLAG_INIT;

// hf_input has been removed, use i-weight and q-weight in individual channels
static char const *Fobos_keys[] = {
  "agc",
  "agc-high-threshold",
  "agc-hold",
  "agc-max-gain",
  "agc-min-gain",
  "clk_source",
  "description",
  "device",
  "direct_sampling",
  "ext_clock", // Synonymous with clk_source, more descriptive
  "frequency",
  "library",
  "lna_gain",
  "samprate",
  "serial",
  "stall-timeout",
  "vga_gain",
  NULL
};

// Global variables set by config file options
extern int Verbose;
extern char const *Description;

enum state {
  STOPPED,
  STARTING,
  STOPPING,
  RUNNING
};
struct sdrstate {
  struct frontend *frontend;
  struct fobos_dev_t *dev;
  bool direct_sampling;
  int lna_gain;
  int vga_gain;
  int buff_count;
  int max_buff_count;
  int device;
  float dc_i,dc_q;              // Average I and Q sample values (DC offsets)
  float variance_i, variance_q; // Average powers excludingt DC
  float covariance;             // I*Q covariance excluding DC
  float beta,q_gain;            // I/Q phase and gain balancing
  float rho;                    // sine of phase error
  _Atomic float scale;         // Scale samples for #bits and front end gain; the callback loads it once a block
  pthread_t monitor_thread;
  _Atomic enum state state;

  // A/D overload protection (fobos_agc)
  pthread_t agc_thread;
  pthread_mutex_t gain_lock;   // fobos_gain (a command) and the AGC both set the gain
  int agc_lna;                 // the LNA setting the AGC keeps
  int agc_vga_min,agc_vga_max; // the VGA steps it moves between
  // Measurements the sample callback publishes for the AGC thread
  _Atomic uint64_t samples_in; // complex samples converted
  _Atomic uint64_t clips_all;  // samples at full scale
  _Atomic float power;         // a copy of frontend->if_power
  // Samples at full scale taken at the current gain, in blocks starting at
  // or after fresh_from: the first sample past those already queued in the
  // library's buffers when the gain last changed. clip_lock makes the
  // callback's test and count one step with the AGC's change and reading
  pthread_mutex_t clip_lock;
  uint64_t fresh_from;
  uint64_t clips_fresh;
  float agc_high_threshold;    // dBFS: above it, the gain comes down
  float agc_hold;              // seconds without overload before the gain goes back up
  int64_t stall_timeout;       // ns without samples before radiod gives up on the device; 0: never
};
static float const AGC_HIGH_THRESHOLD = -15; // dBFS
static float const AGC_HOLD = 30;            // seconds
static float const AGC_HEADROOM = 6;         // dB below the threshold before the gain goes up
static float const AGC_STEP_DOWN = 6;        // dB, at least, when the A/D overloads
static float const AGC_STEP_UP = 2;          // dB, one VGA step
static int64_t const AGC_TICK = BILLION / 10;      // the AGC looks every 100 ms
static int64_t const AGC_SETTLE = BILLION / 5;     // after a cut, before another
static int64_t const AGC_RAISE_EVERY = 5 * (int64_t)BILLION; // between steps up
static int64_t const OVERLOAD_REPORT = 60 * (int64_t)BILLION;
static int const AGC_RETRIES = 10;                  // failed gain changes, a tick apart, before
static int64_t const AGC_RETRY_EVERY = 5 * (int64_t)BILLION; // trying only this often
static int const FOBOS_BUFFER_COUNT = 16;    // the library's sample buffers
static int const FOBOS_BUFFER_LENGTH = 65536;

#ifdef RAW
static void fobos_raw_callback(const uint16_t * restrict samples, uint32_t complex_count, void *ctx);
#else
static void rx_callback(float * restrict buf, unsigned buf_length, void * restrict ctx);
#endif
static void *fobos_monitor(void *p);
static void *fobos_agc(void *p);
static bool set_stages(struct sdrstate *sdr,int lna,int vga);
static int lna_dB(int lna);

static int find_serial_position(const char *serials, const char *serialnumcfg) {
  if (serialnumcfg == NULL)
    return -1; // No serial number to search for
  char serials_copy[256];
  strlcpy(serials_copy, serials, sizeof serials_copy);
  char *token = strtok(serials_copy, " "); // Tokenize the space-delimited list
  int position = 0;

  while (token != NULL) {
    if (strcmp(token, serialnumcfg) == 0)
      return position; // Found the serial number
    token = strtok(NULL, " "); // Get the next token
    position++;
  }
  return -1; // Serial number not found
}

int fobos_setup(struct frontend *const frontend, dictionary const * const dictionary, char const *const section) {
  assert(dictionary != NULL);
  config_validate_section(stderr, dictionary, section, Fobos_keys, NULL);
  frontend->isreal = false; // Make sure the right kind of filter gets created!
#ifdef RAW
  frontend->bitspersample = 14; // gain scaling = 2^-14
#else
  // The Fobos apparently provides scaled float samples
  frontend->bitspersample = 1;  // gain scaling = 1
#endif
  frontend->rf_agc = false; // "agc = yes" in tuner mode
  // Read Config Files
  {
    char const *device = config_getstring(dictionary, section, "device", section);
    if (strcasecmp(device, "fobos") != 0)
      return -1; // Leave if not Fobos in the config
  }
  // Get Fobos Library and Driver Version
  char lib_version[32];
  char drv_version[32];
  int result = fobos_rx_get_api_info(lib_version, drv_version);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr,"fobos_rx_get_api_info failed: %s\n",fobos_rx_error_name(result));
    return -1;
  }
  // Look for connected Fobos Devices and fetch serial numbers
  char serialnumlist[256] = {0};
  int fobos_count = fobos_rx_list_devices(serialnumlist);
  if (fobos_count < 1) {
    fprintf(stderr, "No Fobos SDR devices found\n");
    return -1;
  }
  fprintf(stderr, "Found %d Fobos SDR device(s)\n", fobos_count);

  const char *serialnumcfg = config_getstring(dictionary, section, "serial", NULL);
  // If the config specifies a serial number look for it in the list --
  // otherwise assume device 0
  int position = 0;
  if (serialnumcfg != NULL) {
    position = find_serial_position(serialnumlist, serialnumcfg);
    if (position < 0) {
      fprintf(stderr,
              "Serial number '%s' not found in the list of connected Fobos "
              "devices\n",
              serialnumcfg);
      return -1;
    }
  }
  // Open the SDR
  struct sdrstate *const sdr = calloc(1, sizeof(struct sdrstate));
  // Cross-link generic and hardware-specific control structures
  assert(sdr != NULL);
  sdr->frontend = frontend;
  frontend->context = sdr;
  sdr->scale = scale_AD(frontend);
  sdr->buff_count = 0;
  sdr->max_buff_count = 2048;
  sdr->device = position;
  result = fobos_rx_open(&sdr->dev, sdr->device);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr, "Could not open device: %d\n", sdr->device);
    goto quit; // frees sdr, cleans up
  }
  {
    char const *cp = config_getstring(dictionary, section, "description", Description ? Description : "fobos");
    if(cp != NULL){
      strlcpy(frontend->description,cp,sizeof frontend->description);
      Description = cp;
    }
  }
  char hw_revision[32];
  char fw_version[32];
  char manufacturer[32];
  char product[32];
  char serial[32];
  result = fobos_rx_get_board_info(sdr->dev, hw_revision, fw_version, manufacturer, product, serial);
  if (result == FOBOS_ERR_OK) {
    fprintf(stderr, "%s %s serial %s, hardware %s, lib %s, driver %s firmware %s\n",
	    manufacturer,product,serial,hw_revision, lib_version,drv_version,fw_version);
  } else {
    fprintf(stderr, "fobos_rx_get_board_info: %s\n",fobos_rx_error_name(result));
    goto quit;
  }
  // Get Sample Rates offered by the Fobos
  // First call to get the count of sample rates
  unsigned int samplecount = 0; // Initialize sample count
  result = fobos_rx_get_samplerates(sdr->dev, NULL, &samplecount);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr, "fobos_rx_get_samplerates: %s\n", fobos_rx_error_name(result));
    fobos_rx_close(sdr->dev); // Close the device before returning
    goto quit;
  }
  // Allocate memory for the sample rates array
  double *sampvalues = (double *)malloc(samplecount * sizeof(double));
  if (sampvalues == NULL) {
    fprintf(stderr, "Error: Memory allocation failed for sample rates.\n");
    fobos_rx_close(sdr->dev); // Close the device before returning
    goto quit;
  }
  // Second call to fetch the actual sample rates
  result = fobos_rx_get_samplerates(sdr->dev, sampvalues, &samplecount);
  if (result == FOBOS_ERR_OK) {
    fprintf(stderr, "Supported Sample Rates for SDR #%d:", sdr->device);
    for (unsigned int i = 0; i < samplecount; i++) {
      fprintf(stderr, " %'.0f", sampvalues[i]);
    }
    fprintf(stderr, "\n");
  } else {
    fprintf(stderr, "fobos_rx_get_samplerates: %s\n",fobos_rx_error_name(result));
    fobos_rx_close(sdr->dev);
    FREE(sampvalues);
    goto quit;
  }
  FREE(sampvalues);
  // End of fetching sample rates here
  double const requestsample = config_getdouble(dictionary, section, "samprate", 8000000.0);
  bool clk_sourcecfg = config_getboolean(dictionary, section, "clk_source", 0);
  clk_sourcecfg = config_getboolean(dictionary, section, "ext_clock", clk_sourcecfg);
  // Set the Actual Sample Rate
  double samprate_actual = 0.0;
  result = fobos_rx_set_samplerate(sdr->dev, requestsample, &samprate_actual);
  if (result == FOBOS_ERR_OK) {
    frontend->samprate = samprate_actual;
  } else {
    fprintf(stderr, "fobos_rx_set_samplerate(%lf): %s\n",requestsample, fobos_rx_error_name(result));
    fobos_rx_close(sdr->dev);
    goto quit;
  }
  // Set Direct Sampling
  sdr->direct_sampling = config_getboolean(dictionary, section, "direct_sampling", 0);
  result = fobos_rx_set_direct_sampling(sdr->dev, sdr->direct_sampling);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr, "fobos_rx_set_direct_sampling(%d): %s\n", sdr->direct_sampling, fobos_rx_error_name(result));
    goto quit;
  }
  frontend->min_IF = -0.47 * frontend->samprate;
  frontend->max_IF = 0.47 * frontend->samprate;
  if(sdr->direct_sampling){
    // With -40 dBm @ 15 MHz on B input and nothing on A input,
    // A/D reads -42.2 dBm
    // So level_cal = -0.8 dB gives (average) input of -43.0 dBm
    // Note I flipped the sign convention on rf_level_cal to have units of dBm/FS
    // ie, how many dBm on the input gives 0 dBFS
    frontend->frequency = 0;
    frontend->rf_gain = 0;
    frontend->rf_atten = 0;
    frontend->rf_level_cal = -0.8;
  } else {
    const char *frequencycfg = config_getstring(dictionary, section, "frequency", "100m0");
    // Set Frequency
    double const init_frequency = parse_frequency(frequencycfg, false);
    double frequency_actual = 0.0;
    // Wow, a library API that returns the *actual* tuner frequency. Bravo!
    int result = fobos_rx_set_frequency(sdr->dev, init_frequency, &frequency_actual);
    if (result != FOBOS_ERR_OK) {
      fprintf(stderr, "fobos_rx_set_frequency(freq=%lf): %s\n",init_frequency, fobos_rx_error_name(result));
      fobos_rx_close(sdr->dev);
      goto quit;
    }
    frontend->frequency = frequency_actual;
    sdr->lna_gain = config_getint(dictionary, section, "lna_gain", 0);
    sdr->vga_gain = config_getint(dictionary, section, "vga_gain", 0);
    // These gains are not used in direct sample mode; the MAX2830 is bypassed
    // Set LNA Gain 0..3
    // MAX2830 datasheet, p21: 11 => max gain, 10 => -16 dB, 0X => -33 dB
    result = fobos_rx_set_lna_gain(sdr->dev, sdr->lna_gain);
    if (result != FOBOS_ERR_OK) {
      fprintf(stderr, "fobos_rx_set_lna_gain(%d): %s\n",sdr->lna_gain, fobos_rx_error_name(result));
      goto quit;
    }
    // Get VGA Gain 0..31
    // MAX2830 datasheet, p21: 2 dB steps, 0-62 dB
    result = fobos_rx_set_vga_gain(sdr->dev, sdr->vga_gain);
    if (result != FOBOS_ERR_OK) {
      fprintf(stderr, "fobos_rx_set_vga_gain(%d): %s\n",sdr->vga_gain, fobos_rx_error_name(result));
      goto quit;
    }
    frontend->rf_gain = 2 * sdr->vga_gain + lna_dB(sdr->lna_gain);
    frontend->rf_atten = 0;
    frontend->rf_level_cal = -7.1; // very rough approximation, needs to be measured
    frontend->rf_agc = config_getboolean(dictionary, section, "agc", false);
    // The AGC keeps the LNA as set, and moves the VGA between the steps
    // the limits (total gain, dB) allow: the lowest at or above the
    // minimum, the highest at or below the maximum
    float const max_gain = config_getfloat(dictionary, section, "agc-max-gain", frontend->rf_gain);
    float const min_gain = config_getfloat(dictionary, section, "agc-min-gain", 0);
    sdr->agc_high_threshold = config_getfloat(dictionary, section, "agc-high-threshold", AGC_HIGH_THRESHOLD);
    sdr->agc_hold = config_getfloat(dictionary, section, "agc-hold", AGC_HOLD);
    sdr->agc_lna = sdr->lna_gain;
    int const lna = lna_dB(sdr->lna_gain);
    // The VGA steps inside the limits: none, and the AGC stays off
    // Checked before conversion: a NaN, infinite or out of range value would
    // make the integer conversion undefined
    bool const limits = isfinite(min_gain) && isfinite(max_gain)
      && min_gain <= lna + 62 && max_gain >= lna && min_gain <= max_gain;
    int vga_min = 0, vga_max = 31;
    if(limits){
      vga_min = min_gain <= lna ? 0 : (int)ceilf((min_gain - lna) / 2);         // 1..31
      vga_max = max_gain >= lna + 62 ? 31 : (int)floorf((max_gain - lna) / 2);  // 0..30
    }
    if(!isfinite(sdr->agc_high_threshold) || sdr->agc_high_threshold >= 0 || sdr->agc_high_threshold < -80){
      fprintf(stderr,"fobos: agc-high-threshold %.1f is not a level from -80 to below 0 dBFS, using %.1f\n",
	      sdr->agc_high_threshold,AGC_HIGH_THRESHOLD);
      sdr->agc_high_threshold = AGC_HIGH_THRESHOLD;
    }
    if(!isfinite(sdr->agc_hold) || sdr->agc_hold < 0 || sdr->agc_hold > 86400){
      fprintf(stderr,"fobos: agc-hold %.1f is not 0 to 86400 seconds, using %.0f\n",sdr->agc_hold,AGC_HOLD);
      sdr->agc_hold = AGC_HOLD;
    }
    if(frontend->rf_agc && (!limits || vga_min > vga_max)){
      fprintf(stderr,"fobos: agc-min-gain %.1f and agc-max-gain %.1f leave no VGA step with lna_gain %d (%d to %d dB): AGC off\n",
	      min_gain,max_gain,sdr->lna_gain,lna,lna + 62);
      frontend->rf_agc = false;
    }
    sdr->agc_vga_min = vga_min;
    sdr->agc_vga_max = vga_max;
    if(frontend->rf_agc){
      fprintf(stderr,"fobos AGC on: lna_gain %d, vga_gain %d to %d (gain %d to %d dB), down above %.1f dBFS or at A/D full scale, up after %.0f s\n",
	      sdr->agc_lna,vga_min,vga_max,lna + 2 * vga_min,lna + 2 * vga_max,
	      sdr->agc_high_threshold,sdr->agc_hold);
      // Start inside the limits
      int const start = sdr->vga_gain < vga_min ? vga_min : sdr->vga_gain > vga_max ? vga_max : sdr->vga_gain;
      if(start != sdr->vga_gain){
	result = fobos_rx_set_vga_gain(sdr->dev, start);
	if (result != FOBOS_ERR_OK) {
	  fprintf(stderr, "fobos_rx_set_vga_gain(%d): %s\n",start, fobos_rx_error_name(result));
	  goto quit;
	}
	fprintf(stderr,"fobos AGC: vga_gain %d is outside that, starting at %d\n",sdr->vga_gain,start);
	sdr->vga_gain = start;
	frontend->rf_gain = 2 * sdr->vga_gain + lna;
      }
    }
  }
  if(sdr->direct_sampling && config_getboolean(dictionary, section, "agc", false))
    fprintf(stderr,"fobos: agc ignored in direct sample mode (no gain control)\n");
  pthread_mutex_init(&sdr->gain_lock,NULL);
  pthread_mutex_init(&sdr->clip_lock,NULL);
  {
    // The Fobos can stop sending samples without leaving the bus (a firmware
    // or USB stall), and radiod would wait for ever. With no samples for
    // this long it exits with EX_IOERR (74), for a wrapper to reset the
    // device and start it again. 0 never gives up
    double const t = config_getdouble(dictionary, section, "stall-timeout", 5);
    sdr->stall_timeout = isfinite(t) && t > 0 && t <= 3600 ? (int64_t)(t * BILLION) : 0;
    if(!(isfinite(t) && t >= 0 && t <= 3600))
      fprintf(stderr,"fobos: stall-timeout %g is not 0 to 3600 s: off\n",t);
  }
  // Set Clock Source
  result = fobos_rx_set_clk_source(sdr->dev, clk_sourcecfg);
  if (result != FOBOS_ERR_OK) {
      fprintf(stderr, "fobos_rx_set_clk_source(%d): %s\n",clk_sourcecfg, fobos_rx_error_name(result));
    goto quit;
  }
  fprintf(stderr,"samprate %'lf Hz, tuner %'.3lf Hz, lna_gain %d (%d dB) vga_gain %d (%d dB)\n",
	  frontend->samprate,
	  frontend->frequency,
	  sdr->lna_gain,
	  lna_dB(sdr->lna_gain),
	  sdr->vga_gain,
	  sdr->vga_gain * 2);
  // SDR is open here
  return 0;
 quit:
  free(sdr);
  frontend->context = NULL;
  return -1;
} // End of Setup

/* command to set analog gain. Turn off AGC if it was on
  MAX2830 datasheet: vga gain 0-63 dB in 2 dB steps (0x00 - 0x1F)
  lna gain: 00 -> -33 dB; 10 -> -16 dB; 11 -> 0 dB
  NF improves with higher lna gain setting, so use it as soon as possible
  Absolute receiver gain not yet measured, but the Fobos block diagram has
  1. QPL9547 LNA (19.5 dB, 0-2.4 GHz) or 9504 (21.6 dB, 2.4-6.0 GHz)
  2. RFFC5072 mixer (-2 dB)
  3. SKY6540S "VGA" (actually fixed at 14-15 dB depending on -11 or -20 version)
  4. MAX 2830 (what we're apparently programming) ~ 1-99 dB
  5. Linear LTC2143 A/D: 1V p-p or 2V p-p
*/
// The gain the LNA setting adds: 0 and 1 -33 dB, 2 -16 dB, 3 0 dB (MAX2830 p21),
// counted from the lowest
static int lna_dB(int lna){
  return lna == 3 ? 33 : lna == 2 ? 16 : 0;
}

// Set the MAX2830's LNA (0-3) and VGA (0-31, 2 dB steps), with gain_lock
// held; a stage already so isn't written. The A/D scaling follows so that
// levels in dBm stay right. False if a write failed (that stage is as it was)
static bool set_stages(struct sdrstate *sdr,int lna,int vga){
  struct frontend * const frontend = sdr->frontend;
  bool ok = true, changed = false;
  if(lna != sdr->lna_gain){
    int const result = fobos_rx_set_lna_gain(sdr->dev, lna);
    if (result != FOBOS_ERR_OK){
      fprintf(stderr, "fobos_rx_set_lna_gain(%d): %s\n",lna, fobos_rx_error_name(result));
      ok = false;
    } else {
      sdr->lna_gain = lna;
      changed = true;
    }
  }
  if(vga != sdr->vga_gain){
    int const result = fobos_rx_set_vga_gain(sdr->dev, vga);
    if (result != FOBOS_ERR_OK){
      fprintf(stderr, "fobos_rx_set_vga_gain(%d): %s\n",vga, fobos_rx_error_name(result));
      ok = false;
    } else {
      sdr->vga_gain = vga;
      changed = true;
    }
  }
  if(!changed)
    return ok; // the gain is as it was: so are the scaling and the clip cutoff
  frontend->rf_gain = 2 * sdr->vga_gain + lna_dB(sdr->lna_gain);
  atomic_store_explicit(&sdr->scale, scale_AD(frontend), memory_order_relaxed);
  // Samples up to those now in the library's buffers (and one more buffer,
  // for the device's) were taken at the old gain
  pthread_mutex_lock(&sdr->clip_lock);
  sdr->fresh_from = atomic_load(&sdr->samples_in) + (uint64_t)(FOBOS_BUFFER_COUNT + 1) * FOBOS_BUFFER_LENGTH;
  pthread_mutex_unlock(&sdr->clip_lock);
  return ok;
}

// A gain command: turns the AGC off, and sets the total gain LNA first (the
// noise figure is better with more LNA gain), then the VGA; rounds down
float fobos_gain(struct frontend * const frontend, float gain){
  assert(frontend != NULL);
  struct sdrstate * const sdr = (struct sdrstate *)frontend->context;
  if(sdr->direct_sampling)
    return 0; // the MAX2830 is bypassed
  if(gain < 0)
    gain = 0;
  int const lna = gain >= 33 ? 3 : gain >= 16 ? 2 : 0;
  int vga = (int)((gain - lna_dB(lna)) / 2);
  if(vga > 31)
    vga = 31;
  pthread_mutex_lock(&sdr->gain_lock);
  if(frontend->rf_agc)
    fprintf(stderr,"manual gain setting, turning off AGC\n");
  frontend->rf_agc = false; // under the lock: the AGC can't then overwrite this
  set_stages(sdr,lna,vga);
  float const set = frontend->rf_gain;
  pthread_mutex_unlock(&sdr->gain_lock);
  return set;
}

// A/D overload protection. It moves only the VGA, keeping the configured LNA
// setting, between the steps agc-min-gain and agc-max-gain allow. Every
// 100 ms: samples at full scale, or a level above agc-high-threshold, bring
// the gain down at once (at least 6 dB, enough to put the level AGC_HEADROOM
// below the threshold). After agc-hold seconds with neither, it goes up one
// step (2 dB) every 5 s while that keeps the level AGC_HEADROOM below. With
// the AGC off, or in direct sample mode, overloads are still reported, once a
// minute at most.
// The sample callback measures the level and counts samples at full scale,
// and publishes them (atomics, clip_lock); the gain state is read under
// gain_lock
static void *fobos_agc(void *p){
  struct sdrstate * const sdr = (struct sdrstate *)p;
  struct frontend * const frontend = sdr->frontend;
  pthread_setname("fobos-agc");
  uint64_t last_all = atomic_load(&sdr->clips_all);
  pthread_mutex_lock(&sdr->clip_lock);
  uint64_t last_fresh = sdr->clips_fresh;
  pthread_mutex_unlock(&sdr->clip_lock);
  uint64_t report_clips = 0;
  int64_t now = gps_time_ns();
  int64_t report_time = now, last_change = now - AGC_SETTLE, quiet_since = now;
  uint64_t last_samples = 0;
  int64_t samples_moved = now; // when the sample count last advanced
  int failures = 0;      // gain changes the device refused in a row
  int64_t last_failure = 0;
  enum state s;
  while((s = atomic_load(&sdr->state)) == RUNNING || s == STARTING){
    struct timespec const tick = {.tv_sec = 0, .tv_nsec = AGC_TICK};
    nanosleep(&tick,NULL);
    now = gps_time_ns();
    // Watchdog: samples have come, and stopped (a clean shutdown ends this loop first)
    uint64_t const samples = atomic_load(&sdr->samples_in);
    if(samples != last_samples){
      last_samples = samples;
      samples_moved = now;
    } else if(sdr->stall_timeout != 0 && atomic_load(&sdr->state) == RUNNING
	      && now - samples_moved > (samples != 0 ? sdr->stall_timeout : 3 * sdr->stall_timeout)){
      // Stopped, or (given longer to start) never began
      fprintf(stderr,"fobos: no samples for %.0f s%s: the device has stalled. Exiting (EX_IOERR) to be reset and restarted\n",
	      (double)(now - samples_moved) / BILLION,samples != 0 ? "" : " since starting");
      fflush(stderr);
      _exit(EX_IOERR);
    }
    uint64_t const all = atomic_load(&sdr->clips_all);
    report_clips += all - last_all;
    last_all = all;
    // Overloads at the current gain since the last change: kept through the settle
    pthread_mutex_lock(&sdr->clip_lock);
    uint64_t const fresh = sdr->clips_fresh;
    pthread_mutex_unlock(&sdr->clip_lock);
    uint64_t const pending = fresh - last_fresh;
    float const power = atomic_load_explicit(&sdr->power, memory_order_relaxed);
    float const dBFS = power > 0 ? power2dB(power * scale_ADpower2FS(frontend)) : -INFINITY;
    pthread_mutex_lock(&sdr->gain_lock);
    bool const on = frontend->rf_agc;
    int const old_vga = sdr->vga_gain;
    float const old_gain = frontend->rf_gain;
    pthread_mutex_unlock(&sdr->gain_lock);
    if(now - report_time >= OVERLOAD_REPORT){
      if(report_clips != 0)
	fprintf(stderr,"fobos: A/D overload, %'llu samples at full scale in %.0f s (gain %.0f dB, %.1f dBFS)\n",
		(unsigned long long)report_clips,(double)(now - report_time) / BILLION,old_gain,dBFS);
      report_clips = 0;
      report_time = now;
    }
    if(!on || sdr->direct_sampling)
      continue;
    float const target = sdr->agc_high_threshold - AGC_HEADROOM;
    if(pending != 0 || dBFS > target)
      quiet_since = now;
    if(now - last_change < AGC_SETTLE)
      continue; // the last change hasn't shown in the level yet; overloads stay pending
    int new_vga = old_vga;
    if(pending != 0 || dBFS > sdr->agc_high_threshold){
      float cut = pending != 0 ? AGC_STEP_DOWN : 0;
      if(isfinite(dBFS) && dBFS - target > cut)
	cut = dBFS - target;
      // No more steps than there are: the conversion stays in range
      float const steps = ceilf(cut / 2);
      new_vga = steps >= old_vga - sdr->agc_vga_min ? sdr->agc_vga_min : old_vga - (int)steps;
      if(new_vga < sdr->agc_vga_min)
	new_vga = sdr->agc_vga_min;
    } else if(now - quiet_since >= (int64_t)(sdr->agc_hold * BILLION) && now - last_change >= AGC_RAISE_EVERY
	      && old_vga < sdr->agc_vga_max && dBFS + AGC_STEP_UP <= target){
      new_vga = old_vga + 1;
    }
    if(new_vga == old_vga){
      // At the floor already: nothing to cut. These overloads are dealt
      // with (quiet_since, set above, holds the gain from this one), so they
      // mustn't keep holding it once the band is clear
      last_fresh = fresh;
      continue;
    }
    if(failures >= AGC_RETRIES && now - last_failure < AGC_RETRY_EVERY)
      continue; // the device keeps refusing: try again less often
    pthread_mutex_lock(&sdr->gain_lock);
    bool const still = frontend->rf_agc; // a gain command may just have turned it off
    bool const ok = still && set_stages(sdr,sdr->agc_lna,new_vga);
    float const set = frontend->rf_gain;
    pthread_mutex_unlock(&sdr->gain_lock);
    if(!still)
      continue;
    if(!ok){
      // Not changed: the overloads stay pending, and the next tick tries again
      last_failure = now;
      if(++failures == AGC_RETRIES)
	fprintf(stderr,"fobos AGC: %d gain changes failed in a row, now trying every %.0f s\n",
		failures,(double)AGC_RETRY_EVERY / BILLION);
      continue;
    }
    if(failures >= AGC_RETRIES)
      fprintf(stderr,"fobos AGC: gain change worked again after %d failures\n",failures);
    failures = 0;
    last_change = now;
    // What the callback counts from here on is at the new gain
    pthread_mutex_lock(&sdr->clip_lock);
    last_fresh = sdr->clips_fresh;
    pthread_mutex_unlock(&sdr->clip_lock);
    if(set < old_gain || Verbose) // a cut always; the slow climb back with -v
      fprintf(stderr,"fobos AGC: gain %.0f -> %.0f dB (lna %d, vga %d): %'llu samples at full scale, %.1f dBFS\n",
	      old_gain,set,sdr->agc_lna,new_vga,(unsigned long long)pending,dBFS);
  }
  return NULL;
}
static void *fobos_monitor(void *p) {
  struct sdrstate *const sdr = (struct sdrstate *)p;
  assert(sdr != NULL);
  pthread_setname("fobos-mon");
  int const buffer_count = FOBOS_BUFFER_COUNT;
  int const buffer_length = FOBOS_BUFFER_LENGTH;

  stick_core();
#if RAW
  int result = fobos_rx_read_async_raw(sdr->dev, fobos_raw_callback, sdr, buffer_count, buffer_length);
  if(result != FOBOS_ERR_OK) {
    fprintf(stderr, "fobos_rx_read_async_raw: %s\n",fobos_rx_error_name(result));
    exit(EXIT_FAILURE); // Exit the thread due to an error
  }
#else
  int result = fobos_rx_read_async(sdr->dev, rx_callback, sdr, buffer_count, buffer_length);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr, "fobos_rx_read_async: %s\n",fobos_rx_error_name(result));
    exit(EXIT_FAILURE); // Exit the thread due to an error
  }
#endif
  return NULL; // Return NULL when the thread exits cleanly
}
#ifdef RAW
#if defined(__x86_64__)
#include <immintrin.h>
__attribute__((target("avx2")))
static inline uint64_t hsum_u64x4(__m256i x){
  __m128i lo = _mm256_castsi256_si128(x);
  __m128i hi = _mm256_extracti128_si256(x, 1);
  __m128i sum = _mm_add_epi64(lo, hi);
  return (uint64_t)_mm_cvtsi128_si64(sum) + (uint64_t)_mm_extract_epi64(sum, 1);
}
// The eight individual int32 lanes cannot overflow under the sampcount
// restriction below. Widen before the final horizontal sum so that the
// complete block sum is int64_t.
__attribute__((target("avx2")))
static inline int64_t hsum_i32x8_to_i64(__m256i x){
  __m128i lo32 = _mm256_castsi256_si128(x);
  __m128i hi32 = _mm256_extracti128_si256(x, 1);

  __m256i lo64 = _mm256_cvtepi32_epi64(lo32);
  __m256i hi64 = _mm256_cvtepi32_epi64(hi32);

  __m128i lo = _mm_add_epi64(_mm256_castsi256_si128(lo64), _mm256_extracti128_si256(lo64, 1));
  __m128i hi = _mm_add_epi64(_mm256_castsi256_si128(hi64), _mm256_extracti128_si256(hi64, 1));
  __m128i sum = _mm_add_epi64(lo, hi);
  return (int64_t)_mm_cvtsi128_si64(sum) + (int64_t)_mm_extract_epi64(sum, 1);
}
__attribute__((target("avx2")))
static inline int64_t hsum_i64x4(__m256i x){
  __m128i lo = _mm256_castsi256_si128(x);
  __m128i hi = _mm256_extracti128_si256(x, 1);
  __m128i sum = _mm_add_epi64(lo, hi);
  return (int64_t)_mm_cvtsi128_si64(sum) + (int64_t)_mm_extract_epi64(sum, 1);
}
__attribute__((target("avx2,popcnt")))
static void fobos_convert_avx2(float *restrict dst, uint16_t const *restrict src, uint32_t sampcount,
			       float scale, bool direct_sampling, float beta, float q_gain,
			       int64_t *restrict q_sum_result, int64_t *restrict i_sum_result,
			       uint64_t *restrict q_energy_result, uint64_t *restrict i_energy_result,
			       int64_t *restrict iq_product_result, uint32_t *restrict clip_result){
    assert(dst != NULL);
    assert(src != NULL);
    assert(q_sum_result != NULL);
    assert(i_sum_result != NULL);
    assert(q_energy_result != NULL);
    assert(i_energy_result != NULL);
    assert(iq_product_result != NULL);

    assert(sampcount != 0);
    assert((sampcount & 7u) == 0);
    assert(sampcount <= 1048576u);

#if FOBOS_NONTEMPORAL_STORES
    assert(((uintptr_t)dst & 63u) == 0);
#endif
    __m256i const mask14 = _mm256_set1_epi16(0x3fff);
    __m256i const midpoint = _mm256_set1_epi16(8192);
    __m256i const low_rail = _mm256_set1_epi16(-8192);
    __m256i const high_rail = _mm256_set1_epi16(8191);
    uint32_t clip_bytes = 0; // two per sample at full scale

    __m256i const select_q = _mm256_set1_epi32(0x00000001);
    __m256i const select_i = _mm256_set1_epi32(0x00010000);

    __m256 const vscale = _mm256_set1_ps(scale);
    __m256 const vbeta = _mm256_set1_ps(beta);
    __m256 const vq_scale = _mm256_set1_ps(scale * q_gain);

    __m256i q_sum32 = _mm256_setzero_si256();
    __m256i i_sum32 = _mm256_setzero_si256();

    __m256i q_energy_lo = _mm256_setzero_si256();
    __m256i q_energy_hi = _mm256_setzero_si256();
    __m256i i_energy_lo = _mm256_setzero_si256();
    __m256i i_energy_hi = _mm256_setzero_si256();

    __m256i iq_product_lo = _mm256_setzero_si256();
    __m256i iq_product_hi = _mm256_setzero_si256();

    for (uint32_t n = 0; n < sampcount; n += 8) {
      // Input order: Q0 I0 Q1 I1 ... Q7 I7
      __m256i x16 = _mm256_loadu_si256((__m256i const *)(src + 2 * n));

      x16 = _mm256_and_si256(x16, mask14);
      x16 = _mm256_sub_epi16(x16, midpoint);
      // Samples at full scale: A/D overload
      __m256i const rail = _mm256_or_si256(_mm256_cmpeq_epi16(x16, low_rail), _mm256_cmpeq_epi16(x16, high_rail));
      clip_bytes += (uint32_t)__builtin_popcount((unsigned)_mm256_movemask_epi8(rail));

      __m256i q32 = _mm256_madd_epi16(x16, select_q);
      __m256i i32 = _mm256_madd_epi16(x16, select_i);

      q_sum32 = _mm256_add_epi32(q_sum32, q32);
      i_sum32 = _mm256_add_epi32(i_sum32, i32);

      __m256i q2_32 = _mm256_mullo_epi32(q32, q32);
      __m256i i2_32 = _mm256_mullo_epi32(i32, i32);
      __m256i iq_32 = _mm256_mullo_epi32(i32, q32);

      __m128i q2_lo32 = _mm256_castsi256_si128(q2_32);
      __m128i q2_hi32 = _mm256_extracti128_si256(q2_32, 1);

      __m128i i2_lo32 = _mm256_castsi256_si128(i2_32);
      __m128i i2_hi32 = _mm256_extracti128_si256(i2_32, 1);

      __m128i iq_lo32 = _mm256_castsi256_si128(iq_32);
      __m128i iq_hi32 = _mm256_extracti128_si256(iq_32, 1);

      // Squares are nonnegative, so use unsigned extension
      q_energy_lo = _mm256_add_epi64(q_energy_lo, _mm256_cvtepu32_epi64(q2_lo32));
      q_energy_hi = _mm256_add_epi64(q_energy_hi, _mm256_cvtepu32_epi64(q2_hi32));
      i_energy_lo = _mm256_add_epi64(i_energy_lo, _mm256_cvtepu32_epi64(i2_lo32));
      i_energy_hi = _mm256_add_epi64(i_energy_hi, _mm256_cvtepu32_epi64(i2_hi32));
      // I*Q is signed
      iq_product_lo = _mm256_add_epi64(iq_product_lo, _mm256_cvtepi32_epi64(iq_lo32));
      iq_product_hi = _mm256_add_epi64(iq_product_hi, _mm256_cvtepi32_epi64(iq_hi32));
      __m256 fq = _mm256_cvtepi32_ps(q32);
      __m256 fi = _mm256_cvtepi32_ps(i32);

      if (direct_sampling) {
	// Independent physical inputs: apply common scaling only.
	fq = _mm256_mul_ps(fq, vscale);
	fi = _mm256_mul_ps(fi, vscale);
      } else {
	// Frozen flat I/Q correction:
	// Q' = (Q - beta*I) * q_gain
	// I' = I
	fq = _mm256_sub_ps(fq, _mm256_mul_ps(vbeta, fi));
	fq = _mm256_mul_ps(fq, vq_scale);
	fi = _mm256_mul_ps(fi, vscale);
      }
      // Restore interleaved Q,I output ordering.
      __m256 a = _mm256_unpacklo_ps(fq, fi);
      __m256 b = _mm256_unpackhi_ps(fq, fi);
      __m256 out_lo = _mm256_permute2f128_ps(a, b, 0x20);
      __m256 out_hi = _mm256_permute2f128_ps(a, b, 0x31);
#if FOBOS_NONTEMPORAL_STORES
      _mm256_stream_ps(dst + 2 * n, out_lo);
      _mm256_stream_ps(dst + 2 * n + 8, out_hi);
#else
      _mm256_storeu_ps(dst + 2 * n, out_lo);
      _mm256_storeu_ps(dst + 2 * n + 8, out_hi);
#endif
    }
    *q_sum_result = hsum_i32x8_to_i64(q_sum32);
    *i_sum_result = hsum_i32x8_to_i64(i_sum32);
    *q_energy_result = hsum_u64x4(q_energy_lo) + hsum_u64x4(q_energy_hi);
    *i_energy_result = hsum_u64x4(i_energy_lo) + hsum_u64x4(i_energy_hi);
    *iq_product_result = hsum_i64x4(iq_product_lo) + hsum_i64x4(iq_product_hi);
    *clip_result = clip_bytes / 2;
#if FOBOS_NONTEMPORAL_STORES
    _mm_sfence();
#endif
}
#endif // x86_64
static void fobos_convert_scalar(float *restrict dst, uint16_t const *restrict src, uint32_t sampcount,
				 float scale, bool direct_sampling, float beta, float q_gain,
				 int64_t *restrict q_sum_result, int64_t *restrict i_sum_result,
				 uint64_t *restrict q_energy_result, uint64_t *restrict i_energy_result,
				 int64_t *restrict iq_product_result, uint32_t *restrict clip_result){
  uint32_t clips = 0;
  int64_t q_sum = 0;
  int64_t i_sum = 0;
  uint64_t q_energy = 0;
  uint64_t i_energy = 0;
  int64_t iq_product = 0;

  for (uint32_t n = 0; n < sampcount; n++) {
    int q = (src[2 * n] & 0x3fff) - 8192;
    int i = (src[2 * n + 1] & 0x3fff) - 8192;
    clips += (q == -8192 || q == 8191) + (i == -8192 || i == 8191); // A/D overload

    q_sum += q;
    i_sum += i;

    q_energy += (uint64_t)((int64_t)q * q);
    i_energy += (uint64_t)((int64_t)i * i);
    iq_product += (int64_t)i * q;

    float fq = (float)q;
    float fi = (float)i;

    if (!direct_sampling)
      fq = (fq - beta * fi) * q_gain;

    dst[2 * n] = fq * scale;
    dst[2 * n + 1] = fi * scale;
  }
  *q_sum_result = q_sum;
  *i_sum_result = i_sum;
  *q_energy_result = q_energy;
  *i_energy_result = i_energy;
  *iq_product_result = iq_product;
  *clip_result = clips;
}
static float Slow_alpha; // Smoothing constant for IQ imbalance estimation
// Raw-mode callback from modified libfobos
static void fobos_raw_callback(uint16_t const * restrict samples, uint32_t sampcount, void *ctx){
  struct sdrstate * const sdr = (struct sdrstate *)ctx;
  assert(sdr != NULL);
  struct frontend * restrict frontend = sdr->frontend;
  assert(frontend != NULL);
  assert(sampcount != 0);
  if(!atomic_flag_test_and_set_explicit(&Name_set,memory_order_relaxed))
    pthread_setname("fobos-raw-cb");
  if(Power_alpha == 0){
    // Intialize smoothing parameter for power estimation to give 20 ms time constant
    Power_alpha = -expm1f(-(float)sampcount/ (Blocktime * frontend->samprate));
    assert(Power_alpha > 0 && Power_alpha <= 1);
  }
  if(Slow_alpha == 0){
    // Intialize smoothing parameter for IQ balance estimation to give 5 sec constant
    Slow_alpha = -expm1f(-(float)sampcount/ (5.0 * frontend->samprate));
    assert(Slow_alpha > 0 && Slow_alpha <= 1);
  }
  uint64_t q_energy = 0;
  uint64_t i_energy = 0;
  int64_t dc_i = 0;
  int64_t dc_q = 0;
  int64_t iq_sum = 0;
  uint32_t clips = 0;
  // Cast to real float to help vectorization; complex values are always IQIQ...
  float * const restrict wptr = (float *)frontend->in.input_write_pointer.c;
  assert(wptr != NULL);

  float const scale = atomic_load_explicit(&sdr->scale, memory_order_relaxed);
  // samples contains 2 * complex_count words:
  // samples[] = Q0 I0 Q1 I1 ...
  // Each word contains an unsigned 14-bit offset-binary value in bits 0–13.
#if defined(__x86_64__)
  if(__builtin_cpu_supports("avx2") &&  __builtin_cpu_supports("popcnt"))
    fobos_convert_avx2(wptr, samples, sampcount, scale, sdr->direct_sampling,
			 sdr->beta, sdr->q_gain, &dc_q, &dc_i, &q_energy, &i_energy, &iq_sum, &clips);
  else
#endif
    fobos_convert_scalar(wptr, samples, sampcount, scale, sdr->direct_sampling,
			 sdr->beta, sdr->q_gain, &dc_q, &dc_i, &q_energy, &i_energy, &iq_sum, &clips);

  float const mean_q = (float)dc_q / sampcount;
  float const mean_i = (float)dc_i / sampcount;
  float const variance_q = (float)q_energy / sampcount - mean_q * mean_q;
  float const variance_i = (float)i_energy / sampcount - mean_i * mean_i;
  frontend->if_power += Power_alpha * (variance_q + variance_i - frontend->if_power);

  if(!sdr->direct_sampling){
    // I/Q gain and phase balancing
    // This is ideally done on a resistive termination to get pure thermal noise but can be done
    // on live data with long integration
    sdr->dc_q += Slow_alpha * (mean_q - sdr->dc_q); // smoothed mean Q (DC)
    sdr->dc_i += Slow_alpha * (mean_i - sdr->dc_i); // smoothed mean I (DC)
    sdr->variance_q += Slow_alpha * (variance_q - sdr->variance_q); // mean Q power, excluding DC
    sdr->variance_i += Slow_alpha * (variance_i - sdr->variance_i); // mean I power, excluding DC
    float const covariance = (float)iq_sum / sampcount - mean_i * mean_q;
    sdr->covariance += Slow_alpha * (covariance - sdr->covariance); // mean covariance, excluding DC
    float const residual_q_power = sdr->variance_q - sdr->covariance * sdr->covariance / sdr->variance_i;
    if(sdr->variance_i > 0 && sdr->variance_q > 0 && residual_q_power > 0){
      sdr->beta = sdr->covariance / sdr->variance_i;
      sdr->q_gain = sqrt(sdr->variance_i / residual_q_power);
      frontend->gain_error = sdr->variance_i / sdr->variance_q;
      // Normalized correlation between I and Q, -1 to +1
      frontend->phase_error = sdr->covariance / sqrt(sdr->variance_i * sdr->variance_q);
    }
  }
  write_cfilter(&frontend->in, NULL,sampcount); // Update write pointer, invoke FFT
  // Publish for the AGC thread. Only this thread writes these
  uint64_t const start = atomic_load_explicit(&sdr->samples_in, memory_order_relaxed);
  if(clips != 0){
    frontend->overranges += clips;
    frontend->samp_since_over = 0;
    atomic_fetch_add_explicit(&sdr->clips_all, clips, memory_order_relaxed);
    pthread_mutex_lock(&sdr->clip_lock); // only while overloaded
    if(start >= sdr->fresh_from)
      sdr->clips_fresh += clips;
    pthread_mutex_unlock(&sdr->clip_lock);
  } else
    frontend->samp_since_over += sampcount;
  atomic_store_explicit(&sdr->power, frontend->if_power, memory_order_relaxed);
  atomic_store_explicit(&sdr->samples_in, start + sampcount, memory_order_release);
  frontend->samples += sampcount;
}
#else // not RAW
// Callback for original Fobos floating point mode
// Library does int16->float conversion, DC removal, gain balancing but not phase balancing
static void rx_callback(float const * restrict buf, unsigned sampcount, void *ctx) {
  struct sdrstate * const sdr = (struct sdrstate *)ctx;
  assert(sdr != NULL);
  struct frontend * restrict frontend = sdr->frontend;
  assert(frontend != NULL);
  assert(sampcount != 0);
  if(!atomic_flag_test_and_set_explicit(&Name_set,memory_order_relaxed))
    pthread_setname("fobos-cb");

  if(Power_alpha == 0){
    // Intialize smoothing parameter for power estimation to give 20 ms time constant
    Power_alpha = -expm1f(-(float)sampcount/ (Blocktime * frontend->samprate));
    assert(Power_alpha > 0 && Power_alpha <= 1);
  }
  float in_energy = 0;
  float const scale = atomic_load_explicit(&sdr->scale, memory_order_relaxed);
  // Cast to real float to help vectorization; complex values are always IQIQ...
  float * const restrict wptr = (float *)frontend->in.input_write_pointer.c;
  assert(wptr != NULL);

  for (unsigned int n = 0; n < sampcount*2; n++) { // twice as many real samples as complex
    float const samp = buf[n];
    in_energy += samp * samp;       // Calculate energy of the sample
    wptr[n] = samp * scale;    // Store sample in write pointer buffer
  }
  write_cfilter(&frontend->in, NULL,sampcount); // Update write pointer, invoke FFT
  frontend->samples += sampcount;
  // The library's float samples are already scaled: samples at full scale
  // aren't counted here (only in the raw callback, which RAW selects)
  frontend->samp_since_over += sampcount;

  if (isfinite(in_energy))
    frontend->if_power += Power_alpha * (in_energy / sampcount - frontend->if_power);
}
#endif // RAW
int fobos_startup(struct frontend *const frontend) {
  struct sdrstate *const sdr = (struct sdrstate *)frontend->context;
  while(true){
    enum state s = STOPPED;
    if(atomic_compare_exchange_strong(&sdr->state,&s,STARTING))
      break;
    if(s == RUNNING)
      return 0; // Already running
    usleep(10000); // 10 ms
  }
  sdr->scale = scale_AD(frontend);
  pthread_create(&sdr->monitor_thread, NULL, fobos_monitor, sdr);
  pthread_create(&sdr->agc_thread, NULL, fobos_agc, sdr);
  atomic_store(&sdr->state,RUNNING);
  fprintf(stderr, "fobos read thread running\n");
  return 0;
}

int fobos_shutdown(struct frontend *const frontend) {
  struct sdrstate *const sdr = (struct sdrstate *)frontend->context;
  while(true){
    enum state s = RUNNING;
    if(atomic_compare_exchange_strong(&sdr->state,&s,STOPPING))
      break;
    if(s == STOPPED)
      return 0;
    usleep(10000);
  }
  fobos_rx_cancel_async(sdr->dev);
  pthread_join(sdr->monitor_thread,NULL);
  pthread_join(sdr->agc_thread,NULL);
  atomic_store(&sdr->state,STOPPED);
  fprintf(stderr, "fobos read thread stopped\n");
  return 0;
}

double fobos_tune(struct frontend *const frontend, double const freq) {
  struct sdrstate *const sdr = (struct sdrstate *)frontend->context;
  if(sdr->direct_sampling)
    return 0.0; // No tuning in direct sample mode

  if(Verbose)
    fprintf(stderr, "Trying to tune to: %f\n", freq);
  double frequency_actual = 0.0;
  int result = fobos_rx_set_frequency(sdr->dev, freq, &frequency_actual);
  if (result != FOBOS_ERR_OK) {
    fprintf(stderr, "fobos_rx_set_frequency(freq=%lf): %s\n",freq, fobos_rx_error_name(result));
    return frequency_actual;
  }
  frontend->frequency = frequency_actual;
  return frontend->frequency;
}
