// Talkgroup names for trunked calls, from CSV files such as a
// radioreference.com talkgroup download. Used by monitor and pcmrecord;
// modelled on the repeater database (database.c)
// SPDX-License-Identifier: GPL-3.0-or-later
#define _GNU_SOURCE 1
#include <csv.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include "talkgroup.h"

enum column { COL_IGNORE, COL_ID, COL_TAG, COL_DESCRIPTION, COL_CATEGORY };

static struct {
  char const *name;
  enum column column;
} const Columns[] = {
  {"Decimal", COL_ID},
  {"Alpha Tag", COL_TAG},
  {"Description", COL_DESCRIPTION},
  {"Category", COL_CATEGORY},
};

// One system's talkgroups; the first table (system "") is the general one
struct table {
  char *system;
  struct talkgroup *talkgroups; // sorted by id once loaded
  size_t n, max;
};
static struct table *Tables;
static size_t Ntables;

// One file's parse
struct parse {
  struct table *table;  // where its talkgroups go
  enum column *columns; // what each column of this file holds
  size_t ncolumns, max_columns;
  size_t field;         // column of the next field
  bool header_read;
  bool have_id;         // this line's number is good
  struct talkgroup row;
};

static void set(char **dst, char const *text){
  free(*dst);
  *dst = strdup(text);
}

// Each field
static void field_cb(void *field, size_t size, void *user){
  struct parse * const p = user;
  char const *text = field != NULL ? field : "";
  (void)size;

  if(!p->header_read){
    if(p->field >= p->max_columns){
      p->max_columns += 16;
      p->columns = realloc(p->columns,p->max_columns * sizeof *p->columns);
      if(p->columns == NULL)
	abort();
    }
    if(p->field == 0 && strncmp(text,"\xef\xbb\xbf",3) == 0)
      text += 3; // byte order mark
    enum column column = COL_IGNORE;
    for(size_t i = 0; i < sizeof Columns / sizeof Columns[0]; i++){
      if(strcasecmp(text,Columns[i].name) == 0){
	column = Columns[i].column;
	break;
      }
    }
    p->columns[p->field++] = column;
    p->ncolumns = p->field;
    return;
  }
  enum column const column = p->field < p->ncolumns ? p->columns[p->field] : COL_IGNORE;
  p->field++;
  switch(column){
  case COL_ID:
    {
      char *end = NULL;
      errno = 0;
      unsigned long const id = strtoul(text,&end,10);
      p->have_id = end != text && *end == '\0' && errno == 0 && id <= UINT32_MAX && text[0] != '-';
      p->row.id = (uint32_t)id;
    }
    break;
  case COL_TAG:
    set(&p->row.tag,text);
    break;
  case COL_DESCRIPTION:
    set(&p->row.description,text);
    break;
  case COL_CATEGORY:
    set(&p->row.category,text);
    break;
  default:
    break;
  }
}

// Each end of line
static void line_cb(int eol, void *user){
  struct parse * const p = user;
  (void)eol;

  p->field = 0;
  if(!p->header_read){
    p->header_read = true;
    return;
  }
  if(p->have_id && p->row.tag != NULL && p->row.tag[0] != '\0'){
    struct table * const t = p->table;
    if(t->n >= t->max){
      t->max += 256;
      t->talkgroups = realloc(t->talkgroups,t->max * sizeof *t->talkgroups);
      if(t->talkgroups == NULL)
	abort();
    }
    if(p->row.description == NULL)
      p->row.description = strdup("");
    if(p->row.category == NULL)
      p->row.category = strdup("");
    t->talkgroups[t->n++] = p->row; // the table keeps the strings
  } else {
    free(p->row.tag);
    free(p->row.description);
    free(p->row.category);
  }
  memset(&p->row,0,sizeof p->row);
  p->have_id = false;
}

// The table for a system's talkgroups, made if need be. Names compare
// without case: they come from a config file and a directory name
static size_t find_table(char const *system, bool create){
  for(size_t i = 0; i < Ntables; i++){
    if(strcasecmp(Tables[i].system,system) == 0)
      return i;
  }
  if(!create)
    return SIZE_MAX;
  Tables = realloc(Tables,(Ntables + 1) * sizeof *Tables);
  if(Tables == NULL)
    abort();
  Tables[Ntables] = (struct table){.system = strdup(system)};
  return Ntables++;
}

static int load_file(char const *path, size_t table){
  FILE * const fp = fopen(path,"r");
  if(fp == NULL)
    return -1;
  struct csv_parser csv;
  if(csv_init(&csv,CSV_APPEND_NULL) != 0){
    fclose(fp);
    return -1;
  }
  struct parse parse = {.table = &Tables[table]};
  char buf[4096];
  size_t n;
  while((n = fread(buf,1,sizeof buf,fp)) != 0){
    if(csv_parse(&csv,buf,n,field_cb,line_cb,&parse) != n){
      fprintf(stderr,"%s: %s\n",path,csv_strerror(csv_error(&csv)));
      break;
    }
  }
  csv_fini(&csv,field_cb,line_cb,&parse);
  csv_free(&csv);
  fclose(fp);
  free(parse.columns);
  free(parse.row.tag);
  free(parse.row.description);
  free(parse.row.category);
  return 0;
}

static int compare(void const *a, void const *b){
  struct talkgroup const * const t1 = a, * const t2 = b;
  return t1->id < t2->id ? -1 : t1->id > t2->id;
}

static int name_compare(void const *a, void const *b){
  return strcmp(*(char * const *)a,*(char * const *)b);
}

// Every *.csv in a directory, in name order, so which file wins a duplicate
// doesn't depend on the file system. Each subdirectory of the top one is a
// system's own table
static int load_directory(char const *path, size_t table, bool top){
  DIR * const dir = opendir(path);
  if(dir == NULL)
    return -1;
  char **names = NULL;
  size_t nnames = 0;
  struct dirent const *d;
  while((d = readdir(dir)) != NULL){
    if(d->d_name[0] == '.')
      continue;
    names = realloc(names,(nnames + 1) * sizeof *names);
    if(names == NULL)
      abort();
    if(asprintf(&names[nnames++],"%s/%s",path,d->d_name) < 0)
      abort();
  }
  closedir(dir);
  if(nnames > 0)
    qsort(names,nnames,sizeof *names,name_compare);
  for(size_t i = 0; i < nnames; i++){
    char const * const name = names[i] + strlen(path) + 1;
    size_t const len = strlen(name);
    struct stat st;
    if(stat(names[i],&st) != 0){
      // gone, or a dangling link
    } else if(S_ISDIR(st.st_mode)){
      if(top)
	load_directory(names[i],find_table(name,true),false);
    } else if(len > 4 && strcasecmp(name + len - 4,".csv") == 0){
      if(load_file(names[i],table) != 0)
	fprintf(stderr,"Can't read %s: %s\n",names[i],strerror(errno));
    }
    free(names[i]);
  }
  free(names);
  return 0;
}

int load_talkgroups(char const *path){
  if(path == NULL)
    return -1;
  struct stat st;
  if(stat(path,&st) != 0)
    return -1;
  size_t const general = find_table("",true);
  if(S_ISDIR(st.st_mode) ? load_directory(path,general,true) != 0 : load_file(path,general) != 0)
    return -1;
  // Sorted by number; of a number listed twice, keep the first loaded
  size_t total = 0;
  for(size_t k = 0; k < Ntables; k++){
    struct table * const t = &Tables[k];
    // An insertion sort keeps the order loaded (qsort needn't), and the
    // lists are short and mostly sorted already
    for(size_t i = 1; i < t->n; i++){
      struct talkgroup const tg = t->talkgroups[i];
      size_t j = i;
      for(; j > 0 && compare(&t->talkgroups[j-1],&tg) > 0; j--)
	t->talkgroups[j] = t->talkgroups[j-1];
      t->talkgroups[j] = tg;
    }
    size_t kept = t->n > 0 ? 1 : 0;
    for(size_t i = 1; i < t->n; i++){
      if(t->talkgroups[i].id == t->talkgroups[kept-1].id){
	free(t->talkgroups[i].tag);
	free(t->talkgroups[i].description);
	free(t->talkgroups[i].category);
      } else
	t->talkgroups[kept++] = t->talkgroups[i];
    }
    t->n = kept;
    total += kept;
  }
  return (int)total;
}

struct talkgroup const *lookup_talkgroup(char const *system, char const *mode, char const *dest){
  if(Ntables == 0 || dest == NULL)
    return NULL;
  // A system with its own table uses that alone, whatever its mode. Any
  // other call uses the general table, and only if it's P25: the numbers
  // of other kinds of system are small and collide
  size_t k = system != NULL && system[0] != '\0' ? find_table(system,false) : SIZE_MAX;
  if(k == SIZE_MAX){
    if(mode == NULL || strcmp(mode,"P25") != 0)
      return NULL;
    k = 0;
  }
  struct table const * const t = &Tables[k];
  if(t->n == 0 || dest[0] < '0' || dest[0] > '9')
    return NULL;
  char *end = NULL;
  errno = 0;
  unsigned long const id = strtoul(dest,&end,10);
  if(*end != '\0' || errno != 0 || id > UINT32_MAX)
    return NULL;
  struct talkgroup const key = {.id = (uint32_t)id};
  return bsearch(&key,t->talkgroups,t->n,sizeof *t->talkgroups,compare);
}
