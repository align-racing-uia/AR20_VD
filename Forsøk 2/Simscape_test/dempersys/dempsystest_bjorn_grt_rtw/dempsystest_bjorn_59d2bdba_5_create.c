/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'dempsystest_bjorn/Solver Configuration'.
 */

#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"

PmfMessageId sm_ssci_recordRunTimeError(
  const char *errorId, const char *errorMsg, NeuDiagnosticManager* mgr);

#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))

PmIntVector *pm_create_int_vector(size_t, PmAllocator *);
int_T pm_create_int_vector_fields (PmIntVector *, size_t, PmAllocator *);
int_T pm_create_real_vector_fields(PmRealVector *, size_t, PmAllocator *);
int_T pm_create_char_vector_fields(PmCharVector *, size_t, PmAllocator *);
int_T pm_create_bool_vector_fields(PmBoolVector *, size_t, PmAllocator *);
void pm_rv_equals_rv(const PmRealVector *, const PmRealVector *);
void sm_ssci_setupLoggerFcn_codeGen(const NeDae *dae,
  NeLoggerBuilder *neLoggerBuilder);
int32_T sm_ssci_logFcn_codeGen(const NeDae *dae,
  const NeSystemInput *systemInput,
  PmRealVector *output);
extern const NeAssertData dempsystest_bjorn_59d2bdba_5_assertData[];
extern const NeZCData dempsystest_bjorn_59d2bdba_5_ZCData[];
void dempsystest_bjorn_59d2bdba_5_computeRuntimeParameters(
  const double *runtimeRootVariables,
  double *runtimeParameters);
void dempsystest_bjorn_59d2bdba_5_validateRuntimeParameters(
  const double *runtimeParameters,
  int32_T *assertSatisfactionFlags);
void dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
PmfMessageId dempsystest_bjorn_59d2bdba_5_compDerivs(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_numJacPerturbLoBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_numJacPerturbHiBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_checkDynamics(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  const int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_compOutputsDyn(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_compOutputsKin(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_compOutputs (
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_computeAsmModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_computeSimModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId dempsystest_bjorn_59d2bdba_5_computeZeroCrossings(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
void dempsystest_bjorn_59d2bdba_5_setTargets(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  CTarget *targets);
void dempsystest_bjorn_59d2bdba_5_resetAsmStateVector(const void *mech, double
  *stateVector);
void dempsystest_bjorn_59d2bdba_5_resetSimStateVector(const void *mech, double
  *stateVector);
void dempsystest_bjorn_59d2bdba_5_initializeTrackedAngleState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *modeVector,
  const double *motionData,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_computeDiscreteState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_adjustPosition(
  const void *mech,
  const double *dofDeltas,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_perturbAsmJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_perturbSimJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_perturbFlexibleBodyState(
  const void *mech,
  size_t stageIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_computePosDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void dempsystest_bjorn_59d2bdba_5_computeVelDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void dempsystest_bjorn_59d2bdba_5_projectPartiallyTargetedPos(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *origStateVector,
  int partialType,
  double *stateVector);
void dempsystest_bjorn_59d2bdba_5_propagateMotion(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  double *motionData);
size_t dempsystest_bjorn_59d2bdba_5_computeAssemblyError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData,
  double *error);
size_t dempsystest_bjorn_59d2bdba_5_computeAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  boolean_T forVelocitySatisfaction,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
size_t dempsystest_bjorn_59d2bdba_5_computeFullAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
int dempsystest_bjorn_59d2bdba_5_isInKinematicSingularity(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData);
void dempsystest_bjorn_59d2bdba_5_convertStateVector(
  const void *asmMech,
  const RuntimeDerivedValuesBundle *asmRuntimeDerivedValuesBundle,
  const void *simMech,
  const double *asmStateVector,
  const int *asmModeVector,
  const int *simModeVector,
  double *simStateVector);
void dempsystest_bjorn_59d2bdba_5_constructStateVector(
  const void *mech,
  const double *solverStateVector,
  const double *u,
  const double *uDot,
  const double *discreteStateVector,
  double *fullStateVector);
void dempsystest_bjorn_59d2bdba_5_extractSolverStateVector(
  const void *mech,
  const double *fullStateVector,
  double *solverStateVector);
boolean_T dempsystest_bjorn_59d2bdba_5_isPositionViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
boolean_T dempsystest_bjorn_59d2bdba_5_isVelocityViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
PmfMessageId dempsystest_bjorn_59d2bdba_5_projectStateSim(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *modeVector,
  double *stateVector,
  void *neDiagMgr);
void dempsystest_bjorn_59d2bdba_5_computeConstraintError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  double *error);
void dempsystest_bjorn_59d2bdba_5_resetModeVector(const void *mech, int
  *modeVector);
boolean_T dempsystest_bjorn_59d2bdba_5_hasJointDisToNormModeChange(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector);
PmfMessageId dempsystest_bjorn_59d2bdba_5_performJointDisToNormModeChange(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *prevModeVector,
  const int *modeVector,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
void dempsystest_bjorn_59d2bdba_5_onModeChangedCutJoints(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector,
  double *stateVector);
PmfMessageId dempsystest_bjorn_59d2bdba_5_assemble(const double *u, double *udot,
  double *x,
  NeuDiagnosticManager *neDiagMgr)
{
  (void) x;
  (void) u;
  (void) udot;
  (void) neDiagMgr;
  return NULL;
}

static
  void dae_cg_setParameters_function(const NeDae *dae,
  const NeParameterBundle *paramBundle)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeRootVariables = paramBundle->mRealParameters.mX;
  if (smData->mRuntimeParameterScalars.mN == 0)
    return;
  dempsystest_bjorn_59d2bdba_5_computeRuntimeParameters(
    runtimeRootVariables,
    smData->mRuntimeParameterScalars.mX);
  dempsystest_bjorn_59d2bdba_5_computeAsmRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mAsmRuntimeDerivedValuesBundle);
  dempsystest_bjorn_59d2bdba_5_computeSimRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mSimRuntimeDerivedValuesBundle);
  sm_core_computeRedundantConstraintEquations(
    &dae->mPrivateData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle);

#if 0

  {
    size_t i;
    const size_t n = smData->mSimulationDelegate.mRunTimeEnabledEquations.mSize;
    pmf_printf("\nRuntime Enabled Equations (%lu)\n", n);
    for (i = 0; i < n; ++i)
      pmf_printf("  %2lu:  %d\n", i,
                 smData->mSimulationDelegate.mRunTimeEnabledEquations.mValues[i]);
  }

#endif

}

static
  PmfMessageId dae_cg_pAssert_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeParams = smData->mRuntimeParameterScalars.mX;
  int32_T *assertSatisfactionFlags = daeMethodOutput->mPASSERT.mX;
  (void) systemInput;
  (void) neDiagMgr;
  dempsystest_bjorn_59d2bdba_5_validateRuntimeParameters(
    runtimeParams, assertSatisfactionFlags);
  return NULL;
}

static
  PmfMessageId dae_cg_deriv_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  if (smData->mCachedDerivativesAvailable)
    memcpy(daeMethodOutput->mXP0.mX, smData->mCachedDerivatives.mX,
           133 * sizeof(real_T));
  else
    errorId = dempsystest_bjorn_59d2bdba_5_compDerivs(
      &smData->mSimRuntimeDerivedValuesBundle,
      smData->mSimulationDelegate
      .mRunTimeEnabledEquations.mValues,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mV.mX + 4,
      systemInput->mD.mX,
      daeMethodOutput->mXP0.mX,
      &errorResult,
      neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbLoBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = dempsystest_bjorn_59d2bdba_5_numJacPerturbLoBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mV.mX + 4,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_LO.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbHiBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = dempsystest_bjorn_59d2bdba_5_numJacPerturbHiBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mV.mX + 4,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_HI.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_compOutputs_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  PmfMessageId errorId = NULL;
  NeDaePrivateData *smData = dae->mPrivateData;
  errorId = dempsystest_bjorn_59d2bdba_5_compOutputs(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mV.mX + 4,
    systemInput->mD.mX,
    daeMethodOutput->mY.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_mode_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = dempsystest_bjorn_59d2bdba_5_computeSimModeVector(
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mV.mX + 4,
    daeMethodOutput->mMODE.mX,
    &errorResult,
    neDiagMgr);
  memcpy(smData->mCachedModeVector.mX, daeMethodOutput->mMODE.mX,
         0 * sizeof(int32_T));
  return errorId;
}

static
  PmfMessageId dae_cg_zeroCrossing_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  double errorResult = 0.0;
  return
    dempsystest_bjorn_59d2bdba_5_computeZeroCrossings(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mV.mX + 4,
    systemInput->mD.mX,
    daeMethodOutput->mZC.mX,
    &errorResult,
    neDiagMgr);
}

static
  PmfMessageId dae_cg_project_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    false,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_check_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumConstraintEqns > 0)
    errorId = sm_core_projectState(
      false,
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mD.mX,
      systemInput->mX.mX, neDiagMgr);
  if (errorId == NULL) {
    double result = 0.0;
    errorId = dempsystest_bjorn_59d2bdba_5_checkDynamics(
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mV.mX + 4,
      systemInput->mD.mX,
      systemInput->mM.mX,
      &result, neDiagMgr);
  }

  return errorId;
}

static
  PmfMessageId dae_cg_CIC_MODE_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  const size_t mvSize = smData->mModeVectorSize;
  boolean_T modeChanged = false;
  if (mvSize > 0) {
    errorId = dempsystest_bjorn_59d2bdba_5_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mV.mX + 4,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;

    {
      size_t i;
      for (i = 0; i < mvSize; ++i)
        if (systemInput->mM.mX[i] != smData->mCachedModeVector.mX[i]) {
          modeChanged = true;
          break;
        }
    }
  }

  if (modeChanged) {
    errorId = sm_core_onModeChanged(
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mD.mX,
      smData->mCachedModeVector.mX,
      systemInput->mM.mX,
      systemInput->mX.mX,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  errorId =
    sm_core_projectState(
    true,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 4,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_assemble_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  const SmMechanismDelegate *delegate = &smData->mAssemblyDelegate;
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle =
    &smData->mAsmRuntimeDerivedValuesBundle;
  PmfMessageId errorId = NULL;
  size_t i;
  double errorResult = 0.0;
  const size_t numTargets = 104;
  unsigned int asmStatus = 0;
  double *assemblyFullStateVector = smData->mAssemblyFullStateVector.mX;
  double *simulationFullStateVector = smData->mSimulationFullStateVector.mX;
  (*delegate->mSetTargets)(runtimeDerivedValuesBundle, smData->mTargets);

  {
    const double *u = systemInput->mU.mX;
    const double *uDot = u + smData->mInputVectorSize;
    CTarget *target = smData->mTargets + smData->mNumInternalTargets;
    for (i = 0; i < smData->mNumInputMotionPrimitives; ++i) {
      const size_t inputOffset = smData->mMotionInputOffsets.mX[i];
      (target++)->mValue[0] = u [inputOffset];
      (target++)->mValue[0] = uDot[inputOffset];
    }
  }

  if (smData->mAssemblyModeVector.mN > 0) {
    errorId = dempsystest_bjorn_59d2bdba_5_computeAsmModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mV.mX + 4,
      smData->mAssemblyModeVector.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
  }

  sm_core_computeStateVector(
    delegate, runtimeDerivedValuesBundle, smData->mAssemblyModeVector.mX,
    numTargets, smData->mTargets, assemblyFullStateVector);
  asmStatus = sm_core_checkAssembly(
    delegate, runtimeDerivedValuesBundle, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX,
    NULL, NULL, NULL);
  if (asmStatus != 1) {
    return sm_ssci_recordRunTimeError(
      "sm:compiler:messages:simulationErrors:AssemblyFailure",
      asmStatus == 2 ?
      "Model not assembled due to a position violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
      :
      (asmStatus == 3 ?
       "Model not assembled due to a velocity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
       :
       "Model not assembled due to a singularity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."),
      neDiagMgr);
  }

#if 0

  dempsystest_bjorn_59d2bdba_5_checkTargets(
    &smData->mSimRuntimeDerivedValuesBundle,
    assemblyFullStateVector);

#endif

  if (smData->mModeVectorSize > 0) {
    errorId = dempsystest_bjorn_59d2bdba_5_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 4,
      systemInput->mV.mX + 4,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  (*delegate->mConvertStateVector)(
    NULL, runtimeDerivedValuesBundle, NULL, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX, systemInput->mM.mX,
    simulationFullStateVector);
  for (i = 0; i < smData->mStateVectorSize; ++i)
    systemInput->mX.mX[i] = simulationFullStateVector[smData->
      mStateVectorMap.mX[i]];
  memcpy(systemInput->mD.mX,
         simulationFullStateVector +
         smData->mFullStateVectorSize - smData->mDiscreteStateSize,
         smData->mDiscreteStateSize * sizeof(double));
  return errorId;
}

typedef struct {
  size_t first;
  size_t second;
} SizePair;

static void checkMemAllocStatus(int_T status)
{
  (void) status;
}

static
  PmCharVector cStringToCharVector(const char *src)
{
  const size_t n = strlen(src);
  PmCharVector charVect;
  const int_T status =
    pm_create_char_vector_fields(&charVect, n + 1, pm_default_allocator());
  checkMemAllocStatus(status);
  strcpy(charVect.mX, src);
  return charVect;
}

static
  void initBasicAttributes(NeDaePrivateData *smData)
{
  size_t i;
  smData->mStateVectorSize = 133;
  smData->mFullStateVectorSize = 133;
  smData->mDiscreteStateSize = 0;
  smData->mModeVectorSize = 0;
  smData->mNumZeroCrossings = 0;
  smData->mInputVectorSize = 4;
  smData->mOutputVectorSize = 4;
  smData->mNumConstraintEqns = 48;
  for (i = 0; i < 4; ++i)
    smData->mChecksum[i] = 0;
}

static
  void initStateVector(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T stateVectorMap[133] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
    120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
    130, 131, 132
  };

  const CTarget targets[104] = {
    { 0, 87, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 87, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 97, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 97, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true, 1,
      { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 103, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 103, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 104, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 104, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 105, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 105, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 106, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 106, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 107, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 107, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 108, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 108, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 109, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 109, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 110, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 110, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 111, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 111, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 112, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 112, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 113, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 113, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 114, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 114, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 115, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 115, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 116, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 116, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 117, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 117, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 118, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 118, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 230, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 230, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 240, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 240, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 246, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 246, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 247, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 247, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 248, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 248, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 249, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 249, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 250, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 250, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 251, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 251, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 252, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 252, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 253, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 253, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 254, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 254, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 255, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 255, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 256, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 256, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 257, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 257, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 258, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 258, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 259, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 259, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 260, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 260, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 261, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 261, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 1, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 1, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 2, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 295, 2, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 295, 3, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 295, 3, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 1, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 1, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 2, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 303, 2, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 303, 3, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 303, 3, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 1, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 1, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 2, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 315, 2, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 315, 3, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 315, 3, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 0, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 0, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 1, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 1, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 2, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 323, 2, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 1, 323, 3, false, 0, 0, "1", false, false, +1.000000000000000000e+00, true,
      4, { +1.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 2, 323, 3, false, 0, 0, "1", true, false, +1.000000000000000000e+00, true,
      3, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } }
  };

  const size_t numTargets = 104;
  int_T status;
  size_t i;
  status = pm_create_real_vector_fields(
    &smData->mAssemblyFullStateVector, 133, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mSimulationFullStateVector, 133, alloc);
  checkMemAllocStatus(status);
  status = pm_create_int_vector_fields(
    &smData->mStateVectorMap, smData->mStateVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mStateVectorMap.mX, stateVectorMap,
         smData->mStateVectorSize * sizeof(int32_T));
  smData->mNumInternalTargets = 104;
  smData->mNumInputMotionPrimitives = 0;
  PM_ALLOCATE_ARRAY(smData->mTargets, CTarget, numTargets, alloc);
  for (i = 0; i < numTargets; ++i)
    sm_compiler_CTarget_copy(targets + i, smData->mTargets + i);
}

static void initAsserts(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mNumParamAsserts = 0;
  smData->mParamAssertObjects = NULL;
  smData->mParamAssertPaths = NULL;
  smData->mParamAssertDescriptors = NULL;
  smData->mParamAssertMessages = NULL;
  smData->mParamAssertMessageIds = NULL;
  status = pm_create_bool_vector_fields(
    &smData->mParamAssertIsWarnings, smData->mNumParamAsserts, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumParamAsserts > 0) {
    const NeAssertData *ad = dempsystest_bjorn_59d2bdba_5_assertData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mParamAssertObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertDescriptors,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessages,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessageIds,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumParamAsserts; ++i, ++ad) {
      smData->mParamAssertObjects [i] = cStringToCharVector(ad->mObject );
      smData->mParamAssertPaths [i] = cStringToCharVector(ad->mPath );
      smData->mParamAssertDescriptors[i] = cStringToCharVector(ad->mDescriptor);
      smData->mParamAssertMessages [i] = cStringToCharVector(ad->mMessage );
      smData->mParamAssertMessageIds [i] = cStringToCharVector(ad->mMessageID );
      smData->mParamAssertIsWarnings.mX[i] = ad->mIsWarn;
    }
  }
}

static
  void initModeVector(NeDaePrivateData *smData)
{
  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mAssemblyModeVector, 0,
      pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mAssemblyModeVector.mN; ++i)
      smData->mAssemblyModeVector.mX[i] = 0;
  }

  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mCachedModeVector, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mModeVectorSize; ++i)
      smData->mCachedModeVector.mX[i] = 0;
  }
}

static void initZeroCrossings(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mZeroCrossingObjects = NULL;
  smData->mZeroCrossingPaths = NULL;
  smData->mZeroCrossingDescriptors = NULL;
  status = pm_create_int_vector_fields(
    &smData->mZeroCrossingTypes, 0, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumZeroCrossings > 0) {
    const NeZCData *zcd = dempsystest_bjorn_59d2bdba_5_ZCData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingDescriptors,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumZeroCrossings; ++i, ++zcd) {
      smData->mZeroCrossingObjects [i] = cStringToCharVector(zcd->mObject);
      smData->mZeroCrossingPaths [i] = cStringToCharVector(zcd->mPath );
      smData->mZeroCrossingDescriptors[i] = cStringToCharVector(zcd->mDescriptor);
      smData->mZeroCrossingTypes.mX[i] = zcd->mType;
    }
  }
}

static
  void initVariables(NeDaePrivateData *smData)
{
  const char *varFullPaths[133] = {
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Px.p",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Py.p",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Pz.p",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.Q",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.Q",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.Q",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.Q",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Px.v",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Py.v",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.Pz.v",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.w",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.w",
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.S.w",
    "Subsystem.Spherical_Joint13.S.Q",
    "Subsystem.Spherical_Joint13.S.Q",
    "Subsystem.Spherical_Joint13.S.Q",
    "Subsystem.Spherical_Joint13.S.Q",
    "Subsystem.Spherical_Joint13.S.w",
    "Subsystem.Spherical_Joint13.S.w",
    "Subsystem.Spherical_Joint13.S.w",
    "Subsystem.Spherical_Joint12.S.Q",
    "Subsystem.Spherical_Joint12.S.Q",
    "Subsystem.Spherical_Joint12.S.Q",
    "Subsystem.Spherical_Joint12.S.Q",
    "Subsystem.Spherical_Joint12.S.w",
    "Subsystem.Spherical_Joint12.S.w",
    "Subsystem.Spherical_Joint12.S.w",
    "Subsystem.Spherical_Joint8.S.Q",
    "Subsystem.Spherical_Joint8.S.Q",
    "Subsystem.Spherical_Joint8.S.Q",
    "Subsystem.Spherical_Joint8.S.Q",
    "Subsystem.Spherical_Joint8.S.w",
    "Subsystem.Spherical_Joint8.S.w",
    "Subsystem.Spherical_Joint8.S.w",
    "Rev_rundt_x4.Revolute_Joint.Rz.q",
    "Rev_rundt_x4.Revolute_Joint.Rz.w",
    "Rev_rundt_x3.Revolute_Joint.Rz.q",
    "Rev_rundt_x3.Revolute_Joint.Rz.w",
    "Spherical_Joint18.S.Q",
    "Spherical_Joint18.S.Q",
    "Spherical_Joint18.S.Q",
    "Spherical_Joint18.S.Q",
    "Spherical_Joint18.S.w",
    "Spherical_Joint18.S.w",
    "Spherical_Joint18.S.w",
    "Spherical_Joint21.S.Q",
    "Spherical_Joint21.S.Q",
    "Spherical_Joint21.S.Q",
    "Spherical_Joint21.S.Q",
    "Spherical_Joint21.S.w",
    "Spherical_Joint21.S.w",
    "Spherical_Joint21.S.w",
    "Spherical_Joint20.S.Q",
    "Spherical_Joint20.S.Q",
    "Spherical_Joint20.S.Q",
    "Spherical_Joint20.S.Q",
    "Spherical_Joint20.S.w",
    "Spherical_Joint20.S.w",
    "Spherical_Joint20.S.w",
    "Spherical_Joint22.S.Q",
    "Spherical_Joint22.S.Q",
    "Spherical_Joint22.S.Q",
    "Spherical_Joint22.S.Q",
    "Spherical_Joint22.S.w",
    "Spherical_Joint22.S.w",
    "Spherical_Joint22.S.w",
    "Spherical_Joint16.S.Q",
    "Spherical_Joint16.S.Q",
    "Spherical_Joint16.S.Q",
    "Spherical_Joint16.S.Q",
    "Spherical_Joint16.S.w",
    "Spherical_Joint16.S.w",
    "Spherical_Joint16.S.w",
    "Spherical_Joint28.S.Q",
    "Spherical_Joint28.S.Q",
    "Spherical_Joint28.S.Q",
    "Spherical_Joint28.S.Q",
    "Spherical_Joint28.S.w",
    "Spherical_Joint28.S.w",
    "Spherical_Joint28.S.w",
    "Spherical_Joint26.S.Q",
    "Spherical_Joint26.S.Q",
    "Spherical_Joint26.S.Q",
    "Spherical_Joint26.S.Q",
    "Spherical_Joint26.S.w",
    "Spherical_Joint26.S.w",
    "Spherical_Joint26.S.w",
    "Spherical_Joint27.S.Q",
    "Spherical_Joint27.S.Q",
    "Spherical_Joint27.S.Q",
    "Spherical_Joint27.S.Q",
    "Spherical_Joint27.S.w",
    "Spherical_Joint27.S.w",
    "Spherical_Joint27.S.w",
    "Subsystem.Rev_rundt_x1.Revolute_Joint.Rz.q",
    "Subsystem.Rev_rundt_x1.Revolute_Joint.Rz.w",
    "Subsystem.Rev_rundt_x2.Revolute_Joint.Rz.q",
    "Subsystem.Rev_rundt_x2.Revolute_Joint.Rz.w",
    "Subsystem.Spherical_Joint.S.Q",
    "Subsystem.Spherical_Joint.S.Q",
    "Subsystem.Spherical_Joint.S.Q",
    "Subsystem.Spherical_Joint.S.Q",
    "Subsystem.Spherical_Joint.S.w",
    "Subsystem.Spherical_Joint.S.w",
    "Subsystem.Spherical_Joint.S.w",
    "Subsystem.Spherical_Joint6.S.Q",
    "Subsystem.Spherical_Joint6.S.Q",
    "Subsystem.Spherical_Joint6.S.Q",
    "Subsystem.Spherical_Joint6.S.Q",
    "Subsystem.Spherical_Joint6.S.w",
    "Subsystem.Spherical_Joint6.S.w",
    "Subsystem.Spherical_Joint6.S.w",
    "Subsystem.Spherical_Joint4.S.Q",
    "Subsystem.Spherical_Joint4.S.Q",
    "Subsystem.Spherical_Joint4.S.Q",
    "Subsystem.Spherical_Joint4.S.Q",
    "Subsystem.Spherical_Joint4.S.w",
    "Subsystem.Spherical_Joint4.S.w",
    "Subsystem.Spherical_Joint4.S.w",
    "Subsystem.Spherical_Joint5.S.Q",
    "Subsystem.Spherical_Joint5.S.Q",
    "Subsystem.Spherical_Joint5.S.Q",
    "Subsystem.Spherical_Joint5.S.Q",
    "Subsystem.Spherical_Joint5.S.w",
    "Subsystem.Spherical_Joint5.S.w",
    "Subsystem.Spherical_Joint5.S.w",
    "Subsystem.Spherical_Joint14.S.Q",
    "Subsystem.Spherical_Joint14.S.Q",
    "Subsystem.Spherical_Joint14.S.Q",
    "Subsystem.Spherical_Joint14.S.Q",
    "Subsystem.Spherical_Joint14.S.w",
    "Subsystem.Spherical_Joint14.S.w",
    "Subsystem.Spherical_Joint14.S.w"
  };

  const char *varObjects[133] = {
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Wheel Rest/6-DOF Joint1",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint13",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint12",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Subsystem/Spherical Joint8",
    "dempsystest_bjorn/Rev rundt x4/Revolute Joint",
    "dempsystest_bjorn/Rev rundt x4/Revolute Joint",
    "dempsystest_bjorn/Rev rundt x3/Revolute Joint",
    "dempsystest_bjorn/Rev rundt x3/Revolute Joint",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint18",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint21",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint20",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint22",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint16",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint28",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint26",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Spherical Joint27",
    "dempsystest_bjorn/Subsystem/Rev rundt x1/Revolute Joint",
    "dempsystest_bjorn/Subsystem/Rev rundt x1/Revolute Joint",
    "dempsystest_bjorn/Subsystem/Rev rundt x2/Revolute Joint",
    "dempsystest_bjorn/Subsystem/Rev rundt x2/Revolute Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint6",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint4",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint5",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14",
    "dempsystest_bjorn/Subsystem/Spherical Joint14"
  };

  smData->mNumVarScalars = 133;
  smData->mVarFullPaths = NULL;
  smData->mVarObjects = NULL;
  if (smData->mNumVarScalars > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(smData->mVarFullPaths, PmCharVector, 133, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarObjects, PmCharVector, 133, alloc);
    for (s = 0; s < smData->mNumVarScalars; ++s) {
      smData->mVarFullPaths[s] = cStringToCharVector(varFullPaths[s]);
      smData->mVarObjects[s] = cStringToCharVector(varObjects[s]);
    }
  }
}

static
  void initRuntimeParameters(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  size_t i = 0;
  const int32_T *rtpRootVarRows = NULL;
  const int32_T *rtpRootVarCols = NULL;
  const char **rtpFullPaths = NULL;
  smData->mNumRtpRootVars = 0;
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarRows, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarRows.mX, rtpRootVarRows,
         smData->mNumRtpRootVars * sizeof(int32_T));
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarCols, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarCols.mX, rtpRootVarCols,
         smData->mNumRtpRootVars * sizeof(int32_T));
  smData->mRtpFullPaths = NULL;
  if (smData->mNumRtpRootVars > 0) {
    size_t v;
    PM_ALLOCATE_ARRAY(smData->mRtpFullPaths, PmCharVector, 0, alloc);
    for (v = 0; v < smData->mNumRtpRootVars; ++v) {
      smData->mRtpFullPaths[v] = cStringToCharVector(rtpFullPaths[v]);
    }
  }

  smData->mNumRuntimeRootVarScalars = 0;
  status = pm_create_real_vector_fields(
    &smData->mRuntimeParameterScalars, 0,
    alloc);
  checkMemAllocStatus(status);
  for (i = 0; i < smData->mRuntimeParameterScalars.mN; ++i)
    smData->mRuntimeParameterScalars.mX[i] = 0.0;
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mAsmRuntimeDerivedValuesBundle,
    0,
    0);
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mSimRuntimeDerivedValuesBundle,
    0,
    0);
}

static
  void initIoInfoHelper(
  size_t n,
  const char *portPathsSource[],
  const char *unitsSource[],
  const SizePair dimensions[],
  boolean_T doInputs,
  NeDaePrivateData *smData)
{
  PmCharVector *portPaths = NULL;
  PmCharVector *units = NULL;
  NeDsIoInfo *infos = NULL;
  if (n > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(portPaths, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(units, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(infos, NeDsIoInfo, n, alloc);
    for (s = 0; s < n; ++s) {
      portPaths[s] = cStringToCharVector(portPathsSource[s]);
      units[s] = cStringToCharVector(unitsSource[s]);

      {
        NeDsIoInfo *info = infos + s;
        info->mName = info->mIdentifier = portPaths[s].mX;
        info->mM = dimensions[s].first;
        info->mN = dimensions[s].second;
        info->mUnit = units[s].mX;
      }
    }
  }

  if (doInputs) {
    smData->mNumInputs = n;
    smData->mInputPortPaths = portPaths;
    smData->mInputUnits = units;
    smData->mInputInfos = infos;
  } else {
    smData->mNumOutputs = n;
    smData->mOutputPortPaths = portPaths;
    smData->mOutputUnits = units;
    smData->mOutputInfos = infos;
  }
}

static
  void initIoInfo(NeDaePrivateData *smData)
{
  const char *inputPortPaths[4] = {
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.fzi",
    "Subsystem.Wheel_Rest2.x6_DOF_Joint2.fzi",
    "Wheel_Rest1.x6_DOF_Joint1.fzi",
    "Wheel_Rest4.x6_DOF_Joint2.fzi"
  };

  const char *inputUnits[4] = {
    "m*kg/s^2",
    "m*kg/s^2",
    "m*kg/s^2",
    "m*kg/s^2"
  };

  const SizePair inputDimensions[4] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 }
  };

  const char *outputPortPaths[4] = {
    "Subsystem.Wheel_Rest.x6_DOF_Joint1.vz",
    "Subsystem.Wheel_Rest2.x6_DOF_Joint2.vz",
    "Wheel_Rest1.x6_DOF_Joint1.vz",
    "Wheel_Rest4.x6_DOF_Joint2.vz"
  };

  const char *outputUnits[4] = {
    "m/s",
    "m/s",
    "m/s",
    "m/s"
  };

  const SizePair outputDimensions[4] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 }
  };

  initIoInfoHelper(4, inputPortPaths, inputUnits, inputDimensions,
                   true, smData);
  initIoInfoHelper(4, outputPortPaths, outputUnits, outputDimensions,
                   false, smData);
}

static
  void initInputDerivs(NeDaePrivateData *smData)
{
  const int32_T numInputDerivs[4] = {
    0, 0, 0, 0
  };

  PmAllocator *alloc = pm_default_allocator();
  const int_T status = pm_create_int_vector_fields(
    &smData->mNumInputDerivs, smData->mInputVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mNumInputDerivs.mX, numInputDerivs,
         4 * sizeof(int32_T));
  smData->mInputOrder = 1;
}

static
  void initDirectFeedthrough(NeDaePrivateData *smData)
{
  const boolean_T directFeedthroughVector[4] = {
    false, false, false, false
  };

  const boolean_T directFeedthroughMatrix[16] = {
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false
  };

  PmAllocator *alloc = pm_default_allocator();

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughVector, 4, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughVector.mX, directFeedthroughVector,
           4 * sizeof(boolean_T));
  }

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughMatrix, 16, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughMatrix.mX, directFeedthroughMatrix,
           16 * sizeof(boolean_T));
  }
}

static
  void initOutputDerivProc(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T outputFunctionMap[4] = {
    0, 0, 0, 0
  };

  smData->mOutputFunctionMap = pm_create_int_vector(4, alloc);
  memcpy(smData->mOutputFunctionMap->mX, outputFunctionMap,
         4 * sizeof(int32_T));
  smData->mNumOutputClasses = 1;
  smData->mHasKinematicOutputs = true;
  smData->mHasDynamicOutputs = false;
  smData->mIsOutputClass0Dynamic = false;
  smData->mDoComputeDynamicOutputs = false;
  smData->mCachedDerivativesAvailable = false;

  {
    size_t i = 0;
    const int_T status = pm_create_real_vector_fields(
      &smData->mCachedDerivatives, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mCachedDerivatives.mN; ++i)
      smData->mCachedDerivatives.mX[i] = 0.0;
  }
}

#if 0

static void initializeSizePairVector(const SmSizePair *data,
  SmSizePairVector *vector)
{
  const size_t n = sm_core_SmSizePairVector_size(vector);
  size_t i;
  for (i = 0; i < n; ++i, ++data)
    sm_core_SmSizePairVector_setValue(vector, i, data++);
}

#endif

static
  void initAssemblyDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[21] = {
    { 87, 5 }, { 97, 4 }, { 103, 10 }, { 105, 6 }, { 107, 8 }, { 108, 7 },

    { 109, 9 }, { 113, 12 }, { 114, 13 }, { 115, 11 }, { 230, 14 }, { 240, 15 },

    { 246, 16 }, { 250, 2 }, { 251, 1 }, { 252, 20 }, { 256, 18 }, { 257, 19 },

    { 258, 17 }, { 260, 3 }, { 295, 0 }
  };

  const size_t primitiveIndices[21 + 1] = {
    0, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
    23, 24
  };

  const SmSizePair stateOffsets[24] = {
    { 0, 7 }, { 1, 8 }, { 2, 9 }, { 3, 10 }, { 13, 17 }, { 20, 24 },

    { 27, 31 }, { 34, 35 }, { 36, 37 }, { 38, 42 }, { 45, 49 }, { 52, 56 },

    { 59, 63 }, { 66, 70 }, { 73, 77 }, { 80, 84 }, { 87, 91 }, { 94, 95 },

    { 96, 97 }, { 98, 102 }, { 105, 109 }, { 112, 116 }, { 119, 123 }, { 126,
      130 }
  };

  const SmSizePair dofOffsets[24] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 6 }, { 6, 9 }, { 9, 12 },

    { 12, 15 }, { 15, 16 }, { 16, 17 }, { 17, 20 }, { 20, 23 }, { 23, 26 },

    { 26, 29 }, { 29, 32 }, { 32, 35 }, { 35, 38 }, { 38, 41 }, { 41, 42 },

    { 42, 43 }, { 43, 46 }, { 46, 49 }, { 49, 52 }, { 52, 55 }, { 55, 58 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[4] = {
    34, 36, 94, 96
  };

  const size_t equationsPerConstraint[19] = {
    0, 0, 0, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3, 3
  };

  const size_t dofToVelSlot[58] = {
    7, 8, 9, 10, 11, 12, 17, 18, 19, 24,
    25, 26, 31, 32, 33, 35, 37, 42, 43, 44,
    49, 50, 51, 56, 57, 58, 63, 64, 65, 70,
    71, 72, 77, 78, 79, 84, 85, 86, 91, 92,
    93, 95, 97, 102, 103, 104, 109, 110, 111, 116,
    117, 118, 123, 124, 125, 130, 131, 132
  };

  const size_t constraintDofs[172] = {
    0, 1, 2, 3, 4, 5, 9, 10, 11, 12,
    13, 14, 29, 30, 31, 35, 36, 37, 0, 1,
    2, 3, 4, 5, 9, 10, 11, 12, 13, 14,
    17, 18, 19, 20, 21, 22, 0, 1, 2, 3,
    4, 5, 9, 10, 11, 12, 13, 14, 43, 44,
    45, 49, 50, 51, 15, 29, 30, 31, 32, 33,
    34, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 35, 36, 37,
    38, 39, 40, 29, 30, 31, 35, 36, 37, 38,
    39, 40, 17, 18, 19, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 17, 18, 19, 20, 21, 22,
    23, 24, 25, 29, 30, 31, 43, 44, 45, 49,
    50, 51, 52, 53, 54, 43, 44, 45, 49, 50,
    51, 52, 53, 54, 9, 10, 11, 12, 13, 14,
    6, 7, 8, 9, 10, 11, 12, 13, 14, 6,
    7, 8, 43, 44, 45, 12, 13, 14, 12, 13,
    14, 41, 55, 56, 57, 42, 43, 44, 45, 46,
    47, 48
  };

  const size_t constraintDofOffsets[19 + 1] = {
    0, 18, 36, 54, 61, 74, 83, 92, 95, 104,
    113, 116, 125, 134, 143, 152, 155, 158, 165, 172
  };

  const size_t Jm = 48;
  const size_t Jn = 58;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 58;
  delegate->mDof = 58;
  delegate->mStateSize = 133;
  delegate->mContinuousStateSize = 133;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 21;
  delegate->mNumConstraints = 19;
  delegate->mNumAllConstraintEquations = 48;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 21, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 21 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 24, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 24 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 24, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 24 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 4, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 4 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 1);
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 172, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 172 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 19);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    19, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 58, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 58, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 24, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 24, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 24, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 24, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 24, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 17, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mCvVelOffsets, 24, &zeroSizePair);
  sm_core_SmRealVector_create(
    &scratchpad->mCvAzimuthValues, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 133, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 11545, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 58, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 58, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 483, 0.0);
  delegate->mSetTargets = dempsystest_bjorn_59d2bdba_5_setTargets;
  delegate->mResetStateVector = dempsystest_bjorn_59d2bdba_5_resetAsmStateVector;
  delegate->mInitializeTrackedAngleState =
    dempsystest_bjorn_59d2bdba_5_initializeTrackedAngleState;
  delegate->mComputeDiscreteState =
    dempsystest_bjorn_59d2bdba_5_computeDiscreteState;
  delegate->mAdjustPosition = dempsystest_bjorn_59d2bdba_5_adjustPosition;
  delegate->mPerturbJointPrimitiveState =
    dempsystest_bjorn_59d2bdba_5_perturbAsmJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState = NULL;
  delegate->mComputePosDofBlendMatrix =
    dempsystest_bjorn_59d2bdba_5_computePosDofBlendMatrix;
  delegate->mComputeVelDofBlendMatrix =
    dempsystest_bjorn_59d2bdba_5_computeVelDofBlendMatrix;
  delegate->mProjectPartiallyTargetedPos =
    dempsystest_bjorn_59d2bdba_5_projectPartiallyTargetedPos;
  delegate->mPropagateMotion = dempsystest_bjorn_59d2bdba_5_propagateMotion;
  delegate->mComputeAssemblyError =
    dempsystest_bjorn_59d2bdba_5_computeAssemblyError;
  delegate->mComputeAssemblyJacobian =
    dempsystest_bjorn_59d2bdba_5_computeAssemblyJacobian;
  delegate->mComputeFullAssemblyJacobian =
    dempsystest_bjorn_59d2bdba_5_computeFullAssemblyJacobian;
  delegate->mIsInKinematicSingularity =
    dempsystest_bjorn_59d2bdba_5_isInKinematicSingularity;
  delegate->mConvertStateVector =
    dempsystest_bjorn_59d2bdba_5_convertStateVector;
  delegate->mConstructStateVector = NULL;
  delegate->mExtractSolverStateVector = NULL;
  delegate->mIsPositionViolation = NULL;
  delegate->mIsVelocityViolation = NULL;
  delegate->mProjectStateSim = NULL;
  delegate->mComputeConstraintError = NULL;
  delegate->mResetModeVector = NULL;
  delegate->mHasJointDisToNormModeChange = NULL;
  delegate->mPerformJointDisToNormModeChange = NULL;
  delegate->mOnModeChangedCutJoints = NULL;
  delegate->mMech = NULL;
}

static
  void initSimulationDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[21] = {
    { 87, 5 }, { 97, 4 }, { 103, 10 }, { 105, 6 }, { 107, 8 }, { 108, 7 },

    { 109, 9 }, { 113, 12 }, { 114, 13 }, { 115, 11 }, { 230, 14 }, { 240, 15 },

    { 246, 16 }, { 250, 2 }, { 251, 1 }, { 252, 20 }, { 256, 18 }, { 257, 19 },

    { 258, 17 }, { 260, 3 }, { 295, 0 }
  };

  const size_t primitiveIndices[21 + 1] = {
    0, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
    23, 24
  };

  const SmSizePair stateOffsets[24] = {
    { 0, 7 }, { 1, 8 }, { 2, 9 }, { 3, 10 }, { 13, 17 }, { 20, 24 },

    { 27, 31 }, { 34, 35 }, { 36, 37 }, { 38, 42 }, { 45, 49 }, { 52, 56 },

    { 59, 63 }, { 66, 70 }, { 73, 77 }, { 80, 84 }, { 87, 91 }, { 94, 95 },

    { 96, 97 }, { 98, 102 }, { 105, 109 }, { 112, 116 }, { 119, 123 }, { 126,
      130 }
  };

  const SmSizePair dofOffsets[24] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 6 }, { 6, 9 }, { 9, 12 },

    { 12, 15 }, { 15, 16 }, { 16, 17 }, { 17, 20 }, { 20, 23 }, { 23, 26 },

    { 26, 29 }, { 29, 32 }, { 32, 35 }, { 35, 38 }, { 38, 41 }, { 41, 42 },

    { 42, 43 }, { 43, 46 }, { 46, 49 }, { 49, 52 }, { 52, 55 }, { 55, 58 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[4] = {
    34, 36, 94, 96
  };

  const size_t equationsPerConstraint[19] = {
    0, 0, 0, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3, 3
  };

  const size_t dofToVelSlot[58] = {
    7, 8, 9, 10, 11, 12, 17, 18, 19, 24,
    25, 26, 31, 32, 33, 35, 37, 42, 43, 44,
    49, 50, 51, 56, 57, 58, 63, 64, 65, 70,
    71, 72, 77, 78, 79, 84, 85, 86, 91, 92,
    93, 95, 97, 102, 103, 104, 109, 110, 111, 116,
    117, 118, 123, 124, 125, 130, 131, 132
  };

  const size_t constraintDofs[172] = {
    0, 1, 2, 3, 4, 5, 9, 10, 11, 12,
    13, 14, 29, 30, 31, 35, 36, 37, 0, 1,
    2, 3, 4, 5, 9, 10, 11, 12, 13, 14,
    17, 18, 19, 20, 21, 22, 0, 1, 2, 3,
    4, 5, 9, 10, 11, 12, 13, 14, 43, 44,
    45, 49, 50, 51, 15, 29, 30, 31, 32, 33,
    34, 16, 17, 18, 19, 20, 21, 22, 23, 24,
    25, 26, 27, 28, 29, 30, 31, 35, 36, 37,
    38, 39, 40, 29, 30, 31, 35, 36, 37, 38,
    39, 40, 17, 18, 19, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 17, 18, 19, 20, 21, 22,
    23, 24, 25, 29, 30, 31, 43, 44, 45, 49,
    50, 51, 52, 53, 54, 43, 44, 45, 49, 50,
    51, 52, 53, 54, 9, 10, 11, 12, 13, 14,
    6, 7, 8, 9, 10, 11, 12, 13, 14, 6,
    7, 8, 43, 44, 45, 12, 13, 14, 12, 13,
    14, 41, 55, 56, 57, 42, 43, 44, 45, 46,
    47, 48
  };

  const size_t constraintDofOffsets[19 + 1] = {
    0, 18, 36, 54, 61, 74, 83, 92, 95, 104,
    113, 116, 125, 134, 143, 152, 155, 158, 165, 172
  };

  const size_t Jm = 48;
  const size_t Jn = 58;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 58;
  delegate->mDof = 58;
  delegate->mStateSize = 133;
  delegate->mContinuousStateSize = 133;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 21;
  delegate->mNumConstraints = 19;
  delegate->mNumAllConstraintEquations = 48;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 21, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 21 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 24, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 24 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 24, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 24 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 4, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 4 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 1);
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 172, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 172 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 19);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 58);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 19);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    19, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 58, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 58, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 24, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 24, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 24, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 24, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 24, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 17, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mCvVelOffsets, 24, &zeroSizePair);
  sm_core_SmRealVector_create(
    &scratchpad->mCvAzimuthValues, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 133, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 11545, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 58, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 133, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 58, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 483, 0.0);
  delegate->mSetTargets = NULL;
  delegate->mResetStateVector = dempsystest_bjorn_59d2bdba_5_resetSimStateVector;
  delegate->mInitializeTrackedAngleState = NULL;
  delegate->mComputeDiscreteState = NULL;
  delegate->mAdjustPosition = NULL;
  delegate->mPerturbJointPrimitiveState =
    dempsystest_bjorn_59d2bdba_5_perturbSimJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState =
    dempsystest_bjorn_59d2bdba_5_perturbFlexibleBodyState;
  delegate->mComputePosDofBlendMatrix = NULL;
  delegate->mComputeVelDofBlendMatrix = NULL;
  delegate->mProjectPartiallyTargetedPos = NULL;
  delegate->mPropagateMotion = NULL;
  delegate->mComputeAssemblyError = NULL;
  delegate->mComputeAssemblyJacobian = NULL;
  delegate->mComputeFullAssemblyJacobian = NULL;
  delegate->mIsInKinematicSingularity = NULL;
  delegate->mConvertStateVector = NULL;
  delegate->mConstructStateVector =
    dempsystest_bjorn_59d2bdba_5_constructStateVector;
  delegate->mExtractSolverStateVector =
    dempsystest_bjorn_59d2bdba_5_extractSolverStateVector;
  delegate->mIsPositionViolation =
    dempsystest_bjorn_59d2bdba_5_isPositionViolation;
  delegate->mIsVelocityViolation =
    dempsystest_bjorn_59d2bdba_5_isVelocityViolation;
  delegate->mProjectStateSim = dempsystest_bjorn_59d2bdba_5_projectStateSim;
  delegate->mComputeConstraintError =
    dempsystest_bjorn_59d2bdba_5_computeConstraintError;
  delegate->mResetModeVector = dempsystest_bjorn_59d2bdba_5_resetModeVector;
  delegate->mHasJointDisToNormModeChange =
    dempsystest_bjorn_59d2bdba_5_hasJointDisToNormModeChange;
  delegate->mPerformJointDisToNormModeChange =
    dempsystest_bjorn_59d2bdba_5_performJointDisToNormModeChange;
  delegate->mOnModeChangedCutJoints =
    dempsystest_bjorn_59d2bdba_5_onModeChangedCutJoints;
  delegate->mMech = NULL;
}

static
  void initMechanismDelegates(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T *motionInputOffsets = NULL;
  int_T status = 0;
  initAssemblyDelegate(&smData->mAssemblyDelegate);
  initSimulationDelegate(&smData->mSimulationDelegate);
  status = pm_create_int_vector_fields(
    &smData->mMotionInputOffsets, smData->mNumInputMotionPrimitives, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMotionInputOffsets.mX, motionInputOffsets,
         0 * sizeof(int32_T));
}

static
  void initComputationFcnPtrs(NeDaePrivateData *smData)
{
  smData->mSetParametersFcn = dae_cg_setParameters_function;
  smData->mPAssertFcn = dae_cg_pAssert_method;
  smData->mDerivativeFcn = dae_cg_deriv_method;
  smData->mNumJacPerturbLoBoundsFcn = dae_cg_numJacPerturbLoBounds_method;
  smData->mNumJacPerturbHiBoundsFcn = dae_cg_numJacPerturbHiBounds_method;
  smData->mOutputFcn = dae_cg_compOutputs_method;
  smData->mModeFcn = dae_cg_mode_method;
  smData->mZeroCrossingFcn = dae_cg_zeroCrossing_method;
  smData->mProjectionFcn = dae_cg_project_solve;
  smData->mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve;
  smData->mCheckFcn =
    (smData->mStateVectorSize == 0) ? dae_cg_check_solve : NULL;
  smData->mAssemblyFcn = dae_cg_assemble_solve;
  smData->mSetupLoggerFcn = sm_ssci_setupLoggerFcn_codeGen;
  smData->mLogFcn = sm_ssci_logFcn_codeGen;
  smData->mResidualsFcn = NULL;
  smData->mLinearizeFcn = NULL;
  smData->mGenerateFcn = NULL;
}

static
  void initLiveLinkToSm(NeDaePrivateData *smData)
{
  smData->mLiveSmLink = NULL;
  smData->mLiveSmLink_destroy = NULL;
  smData->mLiveSmLink_copy = NULL;
}

void dempsystest_bjorn_59d2bdba_5_NeDaePrivateData_create(NeDaePrivateData
  *smData)
{
  initBasicAttributes (smData);
  initStateVector (smData);
  initAsserts (smData);
  initModeVector (smData);
  initZeroCrossings (smData);
  initVariables (smData);
  initRuntimeParameters (smData);
  initIoInfo (smData);
  initInputDerivs (smData);
  initDirectFeedthrough (smData);
  initOutputDerivProc (smData);
  initMechanismDelegates (smData);
  initComputationFcnPtrs (smData);
  initLiveLinkToSm (smData);
}
