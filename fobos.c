// Fobos 
//setup
//startup
//tune



#define _GNU_SOURCE 1
#include <assert.h>
#include <pthread.h>
#include <fobos.h>
#include <errno.h>
#include <unistd.h>
#include <iniparser/iniparser.h>
#if defined(linux)
#include <bsd/string.h>
#endif

#include <sysexits.h>
#include <strings.h>
#include "conf.h"
#include "misc.h"
#include "radio.h"
#include "config.h"



#if 0 // Reimplement this someday
// Configurable parameters
// decibel limits for power
static float const DC_alpha = 1.0e-6;  // high pass filter coefficient for DC offset estimates, per sample
static float const AGC_upper = -20;
static float const AGC_lower = -40;
#endif

static float Power_smooth = 0.05; // Calculate this properly someday

// Global variables set by command line options
extern char const *App_path;
extern int Verbose;

// Define the rx_ctx_t structure if not already defined
struct rx_ctx_t {
    uint32_t buff_count;       // Current buffer count
    uint32_t max_buff_count;   // Maximum buffer count to record
    struct fobos_dev_t *dev;   // Device handle
};

struct fobos_dev_t *dev = NULL;

struct sdr {
  struct frontend *frontend;
  struct fobos_dev_t *device;    // Opaque pointer

  int dev;
  char serial[256];

  bool bias; // Bias tee on/off

  // AGC
  bool agc;
  int holdoff_counter; // Time delay when we adjust gains
  int gain;      // Gain passed to manual gain setting
  float scale;         // Scale samples for #bits and front end gain

  // Sample statistics
  //  int clips;  // Sample clips since last reset
  //  float DC;      // DC offset for real samples

  pthread_t read_thread;
};

//static void rx_callback(uint8_t *buf,uint32_t len, void *ctx);
static void rx_callback(float *buf, uint32_t buf_length, void *ctx);


int find_serial_position(const char *serials, const char *serialnumcfg) {
    if (serialnumcfg == NULL) {
        return -1; // No serial number to search for
    }

    char serials_copy[256];
    strncpy(serials_copy, serials, sizeof(serials_copy) - 1);
    serials_copy[sizeof(serials_copy) - 1] = '\0'; // Ensure null termination

    char *token = strtok(serials_copy, " "); // Tokenize the space-delimited list
    int position = 0;

    while (token != NULL) {
        if (strcmp(token, serialnumcfg) == 0) {
            return position; // Found the serial number
        }
        token = strtok(NULL, " "); // Get the next token
        position++;
    }

    return -1; // Serial number not found
}

///////////////////////////////////////////////////////////
int fobos_setup(struct frontend *frontend,dictionary *dictionary,char const *section)
{
    assert(dictionary != NULL);
    struct sdr * const sdr = (struct sdr *)calloc(1,sizeof(struct sdr));
    // Cross-link generic and hardware-specific control structures
    sdr->frontend = frontend;
    frontend->context = sdr;
    
    // Read Config Files
    char const *device = config_getstring(dictionary,section,"device",NULL);
    if(strcasecmp(device,"fobos") != 0) {
      return -1; // Leave if not Fobos in the config
    }
  
    sdr->dev = -1;
    FREE(frontend->description);
    frontend->description = strdup(config_getstring(dictionary,section,"description","fobos"));
    frontend->samprate = config_getdouble(dictionary,section,"samprate",8000000.0);
    const char *serialnumcfg = config_getstring(dictionary, section, "serial", NULL);
    const char *frequencycfg = config_getstring(dictionary, section, "frequency", "100m0");
    int dirsamplecfg = config_getint(dictionary, section, "direct_sampling", 0);
    int lna_gaincfg = config_getint(dictionary, section, "lna_gain", 0); 
    int vga_gaincfg = config_getint(dictionary, section, "vga_gain", 0); 
    int clk_sourcecfg = config_getint(dictionary, section, "clk_source", 0); 
      
    // Get Fobos Library and Driver Version
    int result = 0;
    char lib_version[32];
    char drv_version[32];
    
    result = fobos_rx_get_api_info(lib_version, drv_version);
    if (result != 0)
    {
      printf("Unable to find Fobos Drivers. Please check libfobos is installed.\n");
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
  
    // If the config specifies a serial number look for it in the list -- otherwise assume device 0
    if (serialnumcfg == NULL) {
        // Use the first device by default
        sdr->dev = 0;
    } else {
        int position = find_serial_position(serialnumlist, serialnumcfg);
        if (position >= 0) {
            sdr->dev = position;
        } else {
            printf("Serial number '%s' not found in the list of connected Fobos devices\n", serialnumcfg);
            return -1; 
        }
      }
  
    // Open the SDR
    result = fobos_rx_open(&dev, sdr->dev);
    if (result != 0) {
        fprintf(stderr, "Could not open device: %d\n", sdr->dev);
        return -1; 
    } else {
        char hw_revision[32];
        char fw_version[32];
        char manufacturer[32];
        char product[32];
        char serial[32];
        
        result = fobos_rx_get_board_info(dev, hw_revision, fw_version, manufacturer, product, serial);
        if (result == 0) {
            fprintf(stderr, "--------------------------------------------\n");
            fprintf(stderr, "Library Version:    %s\n", lib_version);
            fprintf(stderr, "Driver Version:     %s\n", drv_version);
            fprintf(stderr, "Hardware Revision:  %s\n", hw_revision);
            fprintf(stderr, "Firmware Version:   %s\n", fw_version);
            fprintf(stderr, "Manufacturer:       %s\n", manufacturer);
            fprintf(stderr, "Product:            %s\n", product);
            fprintf(stderr, "--------------------------------------------\n");
        } else {
            fprintf(stderr, "Error fetching device info from fobos device: %d\n", sdr->dev);
            return -1;
        }
        
        // Get Sample Rates offered by the Fobos
        double *sampvalues = NULL;     // Pointer to hold sample rates
        unsigned int samplecount = 0;  // Initialize sample count
        
        // First call to get the count of sample rates
        result = fobos_rx_get_samplerates(dev, NULL, &samplecount);
        if (result != 0) {
            fprintf(stderr, "Error fetching sample rate count (error code: %d)\n", result);
            fobos_rx_close(dev); // Close the device before returning
            return -1;
        }
        
        // Allocate memory for the sample rates array
        sampvalues = (double *)malloc(samplecount * sizeof(double));
        if (sampvalues == NULL) {
            fprintf(stderr, "Error: Memory allocation failed for sample rates.\n");
            fobos_rx_close(dev); // Close the device before returning
            return -1;
        }
        
        // Second call to fetch the actual sample rates
        result = fobos_rx_get_samplerates(dev, sampvalues, &samplecount);
        if (result == 0) {
            fprintf(stderr, "--------------------------------------------\n");
            fprintf(stderr, "Supported Sample Rates for SDR #%d:\n", sdr->dev);
            for (unsigned int i = 0; i < samplecount; i++) {
                fprintf(stderr, "  %.0f \n", sampvalues[i]);
            }
            fprintf(stderr, "--------------------------------------------\n");
        } else {
            fprintf(stderr, "Error fetching sample rates (error code: %d)\n", result);
            fobos_rx_close(dev);
        return -1;
        }
        // End of fetching sample rates here
        
        // Set Frequency
        double init_frequency = parse_frequency(frequencycfg,false);
        double frequency_actual = 0.0;
        int result = fobos_rx_set_frequency(dev, init_frequency, &frequency_actual);
        if (result != 0) {
         fprintf(stderr, "fobos_rx_set_frequency failed with error code: %d\n", result);
         fobos_rx_close(dev);
         return -1;
        }
        
        // Set Direct Sampling vs. Non
        result = fobos_rx_set_direct_sampling(dev, dirsamplecfg);
        if (result != 0)
        {
            fprintf(stderr, "fobos_rx_set_direct_sampling failed with error code: %d\n", result);
            return -1;
        }
        
        // Set LNA Gain
        result = fobos_rx_set_lna_gain(dev, lna_gaincfg);
        if (result != 0)
        {
            fprintf(stderr, "fobos_rx_set_lna_gain failed with error code: %d\n", result);
            return -1;
        }
        
        // Get VGA Gain
        result = fobos_rx_set_vga_gain(dev, vga_gaincfg);
        if (result != 0)
        {
            fprintf(stderr, "fobos_rx_set_vga_gain failed with error code: %d\n", result);
            return -1;
        }
        
        // Set Clock Source
        result = fobos_rx_set_clk_source(dev, clk_sourcecfg);
        if (result != 0)
        {
            fprintf(stderr, "fobos_rx_set_clk_source failed with error code: %d\n", result);
            return -1;
        }
                
        // SDR is open here
    }
    return 0;     
  } // End of Setup

// 
static void *read_thread(void *arg) {   
       struct rx_ctx_t rx_ctx;
       rx_ctx.buff_count = 0;
       rx_ctx.dev = dev;
       rx_ctx.max_buff_count = 2048; // number of buffers to record
       fprintf(stdout, "Starting asynchronous read\n");
       // Call the Fobos asynchronous read function
       fprintf(stdout, "Starting asynchronous read\n");
       int result = fobos_rx_read_async(dev, rx_callback, &rx_ctx, 16, 65536);
       if (result != 0) {
           // Log the error and exit the thread if the function fails
           fprintf(stderr, "fobos_rx_read_async failed with error code: %d\n", result);
           exit(EX_NOINPUT); // Exit the thread due to an error
       }
   
       return NULL; // Return NULL when the thread exits cleanly
   }


static void rx_callback(float *buf, uint32_t buf_length, void *ctx) {      
  
       // Check if input buffer and context are valid
       if (!buf) {
           fprintf(stderr, "Error: buf is NULL.\n");
           return;
       }
       if (!ctx) {
           fprintf(stderr, "Error: ctx is NULL.\n");
           return;
       }
   
       struct frontend *frontend = (struct frontend *)ctx;
       if (!frontend) {
           fprintf(stderr, "Error: frontend is NULL.\n");
           return;
       }
   
       struct sdr *sdr = (struct sdr *)frontend->context;
       if (!sdr) {
           fprintf(stderr, "Error: sdr is NULL.\n");
           return;
       }
   
       float complex *wptr = frontend->in.input_write_pointer.c;
       if (!wptr) {
           fprintf(stderr, "Error: wptr (write pointer) is NULL.\n");
           return;
       }
   
       fprintf(stdout, "Callback invoked! Buffer length: %u\n", buf_length);
       fprintf(stdout, "frontend: %p, wptr: %p, buf: %p\n", (void *)frontend, (void *)wptr, (void *)buf);
   
       // Process samples
       float energy = 0.0;
       int sampcount = buf_length / 2; // Interleaved I/Q samples
   
       for (int i = 0; i < sampcount; i++) {
           float complex samp;
   
           // Extract I and Q samples from interleaved floats
           __real__ samp = buf[2 * i];       // I-channel
           __imag__ samp = buf[2 * i + 1];   // Q-channel
   
           // Compute energy (magnitude squared)
           energy += cnrmf(samp);
   
           // Scale and store the sample in the write pointer
           wptr[i] = sdr->scale * samp;
       }
   
       // Update frontend state
       frontend->samples += sampcount;
       frontend->timestamp = gps_time_ns();  // Replace with your timestamp function
       write_cfilter(&frontend->in, NULL, sampcount);  // Update write pointer, invoke FFT
   
       // Compute instantaneous and smoothed power
       if (isfinite(energy)) {
           frontend->if_power_instant = energy / sampcount;
           frontend->if_power += Power_smooth * (frontend->if_power_instant - frontend->if_power);
       }
   
       fprintf(stdout, "Callback processing complete. Processed %d samples.\n", sampcount);
   }




int fobos_startup(struct frontend * const frontend){
  struct sdr * const sdr = frontend->context;
  if (!sdr) {
       fprintf(stderr, "Error in fobos_startup: sdr is NULL.\n");
       return;
   }
  pthread_create(&sdr->read_thread,NULL,read_thread,sdr);
  fprintf(stdout,"fobos read thread running\n");
  return 0;
}


double fobos_tune(struct frontend * const frontend,double const freq){
  if(frontend->lock)
    return frontend->frequency;
  struct sdrstate * const sdr = frontend->context;
  return frontend->frequency;
}



