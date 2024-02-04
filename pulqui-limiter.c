/* pulqui-limiter lv2 plugin
// Written by Lucas Cordiviola 12-2023
// No warranties
// See License.txt
*/


#ifdef HAVE_LV2_1_18_6
#include <lv2/core/lv2.h>
#else
#include <lv2/lv2plug.in/ns/lv2core/lv2.h>
#endif

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define PULQUI_URI "https://github.com/Lucarda/pulqui-limiter.lv2"
#define PULQUI_SIZE 4096
#define PULQUI_SCAN_SIZE 8192


typedef struct {
  // Port buffers
  const float* x_thresh;
  const float* input;
  float*       output;
  float x_ramchpositive[PULQUI_SCAN_SIZE];
  float x_ramchnegative[PULQUI_SCAN_SIZE];
  float x_ramch[PULQUI_SIZE];
  float x_bufsignal[PULQUI_SIZE];
  float x_bufsignalout[PULQUI_SIZE];
  float x_bufpulqui[PULQUI_SIZE];
  int x_scanlen, x_len, x_pulquiblock;
  const float* x_makeup; 
  const float* x_bypass;
  float* report_latency;
} Pulqui;


static LV2_Handle
instantiate(const LV2_Descriptor*     descriptor,
            double                    rate,
            const char*               bundle_path,
            const LV2_Feature* const* features)
{
  Pulqui* self = (Pulqui*)calloc(1, sizeof(Pulqui));

  return (LV2_Handle)self;
}


static void
connect_port(LV2_Handle instance, uint32_t port, void* data)
{
  Pulqui* self = (Pulqui*)instance;

  switch (port) {
  case 0:
    self->x_thresh = (const float*)data;
    break;
  case 1:
    self->x_bypass = (const float*)data;
    break;
  case 2:
    self->x_makeup = (const float*)data;
    break;
  case 3:
    self->report_latency = (float*)data;
    break;
  case 4:
    self->input = (const float*)data;
    break;
  case 5:
    self->output = (float*)data;
    break;
  }
}


static void
activate(LV2_Handle instance)
{
    Pulqui* self = (Pulqui*)instance;
    self->x_scanlen = PULQUI_SCAN_SIZE; 
    self->x_len = PULQUI_SIZE;
    *(self->report_latency) = (float)PULQUI_SCAN_SIZE;   
}

/** Define a macro for converting a gain in dB to a coefficient. */
#define DB_CO(g) ((g) > -90.0f ? powf(10.0f, (g)*0.05f) : 0.0f)

static void pq_bee32(Pulqui* self)
{
    int pos;
    int startpos;
    int endpos;
    float peakIEEE;
    startpos = 0;
    endpos = 0;
    pos = 0;

    LOOP:while (pos < self->x_scanlen)
    {
        if ( self->x_ramchpositive[pos] > 0.0001) break;
        pos++;
    }
    startpos = pos;
    peakIEEE = 0;
    while (pos < self->x_scanlen)
    {
        if (self->x_ramchpositive[pos] > peakIEEE) peakIEEE = self->x_ramchpositive[pos];
        if ( self->x_ramchpositive[pos] < 0.0001) break;
        pos++;
    }
    endpos = pos;
    for (pos = startpos; pos < endpos ; pos++)
    {
        self->x_ramchpositive[pos] = peakIEEE;
    }
    //endpos = pos;
    if (pos < self->x_scanlen) goto LOOP;
}

static void pq_bee32_negative(Pulqui* self)
{
    int pos;
    int startpos;
    int endpos;
    float peakIEEE;
    startpos = 0;
    endpos = 0;
    pos = 0;

    LOOP:while (pos < self->x_scanlen)
    {
        if ( self->x_ramchnegative[pos] < -0.0001) break;
        pos++;
    }
    startpos = pos;
    peakIEEE = 0;
    while (pos < self->x_scanlen)
    {
        if (self->x_ramchnegative[pos] < peakIEEE) peakIEEE = self->x_ramchnegative[pos];
        if ( self->x_ramchnegative[pos] > -0.0001) break;
        pos++;
    }
    endpos = pos;
    for (pos = startpos; pos < endpos ; pos++)
    {
        self->x_ramchnegative[pos] = peakIEEE;
    }
    //endpos = pos;
    if (pos < self->x_scanlen) goto LOOP;
}

static void pulqui_tilde_do_pulqui(Pulqui* self)
{
    int i;
    for (i = 0; i < self->x_len; i++)
    {
         self->x_ramchpositive[self->x_len + i] = self->x_ramch[i];
         self->x_ramchnegative[self->x_len + i] = self->x_ramch[i];
    }

    pq_bee32(self);
    pq_bee32_negative(self);

    for (i = 0; i < self->x_len; i++)
    {
        self->x_bufsignalout[i] = self->x_bufsignal[i];
        if (self->x_ramchpositive[i] >  0.0001)
        {
            self->x_bufpulqui[i] = self->x_ramchpositive[i];
        }
        else if (self->x_ramchnegative[i] <  -0.0001)
        {
            self->x_bufpulqui[i] = self->x_ramchnegative[i] * -1;
        }
        else
        {
            self->x_bufpulqui[i] = 1;
        }
    }

    for (i = 0; i < self->x_len; i++)
    {
         self->x_ramchpositive[i] = self->x_ramchpositive[self->x_len + i];
         self->x_ramchnegative[i] = self->x_ramchnegative[self->x_len + i];
    }

    for (i = 0; i < self->x_len; i++)
    {
        self->x_bufsignal[i] = self->x_ramch[i];
    }
}


static void
run(LV2_Handle instance, uint32_t n_samples)
{
  Pulqui* self = (Pulqui*)instance;

  const float        thresh   = *(self->x_thresh);
  const float* const input  = self->input;
  float* const       output = self->output;
  const int          bypass = *(self->x_bypass);
  const int          makeup = *(self->x_makeup);

  const float coefthresh = DB_CO(thresh);
  float f;

  for (int i = 0; i < n_samples; i++)
    {
        self->x_ramch[i + self->x_pulquiblock] = input[i];
        if(bypass)
        {
            output[i] = self->x_bufsignalout[i + self->x_pulquiblock];
        }
        else
        {
            if (self->x_bufpulqui[i + self->x_pulquiblock] > \
            coefthresh)
                f = self->x_bufsignalout[i + self->x_pulquiblock]*\
                (coefthresh / self->x_bufpulqui[i + self->x_pulquiblock]);
            else 
                f = self->x_bufsignalout[i + self->x_pulquiblock];
            if (makeup)
                output[i] = f*(0.998/coefthresh);
            else
            output[i] = f;
        }
    }

    if(self->x_pulquiblock > ((self->x_len - n_samples) - 1))
    {
        pulqui_tilde_do_pulqui(self);
        self->x_pulquiblock = 0;
    }
    else self->x_pulquiblock += n_samples;
}


static void
deactivate(LV2_Handle instance)
{}


static void
cleanup(LV2_Handle instance)
{
  free(instance);
}


static const void*
extension_data(const char* uri)
{
  return NULL;
}


static const LV2_Descriptor descriptor = {PULQUI_URI,
                                          instantiate,
                                          connect_port,
                                          activate,
                                          run,
                                          deactivate,
                                          cleanup,
                                          extension_data};


#undef LV2_SYMBOL_EXPORT
#ifdef _WIN32
#  define LV2_SYMBOL_EXPORT __declspec(dllexport)
#else
#  define LV2_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
#endif
LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
  return index == 0 ? &descriptor : NULL;
}
