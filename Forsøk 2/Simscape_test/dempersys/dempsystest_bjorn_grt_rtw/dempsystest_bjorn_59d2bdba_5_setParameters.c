/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'dempsystest_bjorn/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void dempsystest_bjorn_59d2bdba_5_computeRuntimeParameters(real_T *in, real_T
  *out)
{
  (void) in;
  (void) out;
}

void dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValuesInts(const
  double *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValues(const double
  *rtp, RuntimeDerivedValuesBundle *rtdv)
{
  dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}

void dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValuesInts(const
  double *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValues(const double
  *rtp, RuntimeDerivedValuesBundle *rtdv)
{
  dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}
