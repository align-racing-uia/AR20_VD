% Autogenerated file: Mon Jun  7 14:45:25 2021
function writeCodeInfoFcn


% Load a ComponentInterface object from IR
codeIRInfoStruct = load('tlc\codeIRInfo.mat');
codeInfo = codeIRInfoStruct.codeInfo;





wr = coder.internal.writeDataInterfacesToCodeDescriptor("dempsystest_bjorn",pwd) ;
wr.writeName(codeInfo.Name);


skippedParameters = [];

  tmpParams = [];





    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        


    
    
    
        






    tmpArgs         = [];
tmpActualArgs   = [];
tmpRet          = coder.types.Argument.empty;
tmpActualReturn = RTW.DataInterface.empty;
  

  func_Initialize_Prototype = coder.types.Prototype;
  func_Initialize_Prototype.Arguments  = tmpArgs;
  func_Initialize_Prototype.Return     = tmpRet;
    func_Initialize_Prototype.HeaderFile = ['dempsystest_bjorn', '.h'];
  func_Initialize_Prototype.SourceFile = ['dempsystest_bjorn', '.c'];

  func_Initialize_Interface = RTW.FunctionInterface;
  func_Initialize_Interface.Prototype    = func_Initialize_Prototype;
  func_Initialize_Interface.ActualArgs   = tmpActualArgs;
  func_Initialize_Interface.ActualReturn = tmpActualReturn;

      func_Initialize_Prototype.Name = 'dempsystest_bjorn_initialize';
  time_constant = codeInfo.TimingProperties(getIndexFromTimingInternalId(codeInfo.TimingInternalIds, -1));
      func_Initialize_Interface.Timing = time_constant;
      codeInfo.InitializeFunctions =  [codeInfo.InitializeFunctions, func_Initialize_Interface'];
      func_Initialize_Interface.IsInitializeFunction = true;

          tmpArgs         = [];
tmpActualArgs   = [];
tmpRet          = coder.types.Argument.empty;
tmpActualReturn = RTW.DataInterface.empty;
        

  

  

  func_UpdateContStates_Prototype = coder.types.Prototype;
  func_UpdateContStates_Prototype.Arguments  = tmpArgs;
  func_UpdateContStates_Prototype.Return     = tmpRet;
    func_UpdateContStates_Prototype.HeaderFile = ['dempsystest_bjorn', '.h'];
  func_UpdateContStates_Prototype.SourceFile = ['dempsystest_bjorn', '.c'];

  func_UpdateContStates_Interface = RTW.FunctionInterface;
  func_UpdateContStates_Interface.Prototype    = func_UpdateContStates_Prototype;
  func_UpdateContStates_Interface.ActualArgs   = tmpActualArgs;
  func_UpdateContStates_Interface.ActualReturn = tmpActualReturn;

  time_1 = codeInfo.TimingProperties(getIndexFromTimingInternalId(codeInfo.TimingInternalIds, 1));
      func_UpdateContStates_Interface.Timing = time_1;
        func_UpdateContStates_Prototype.Name = 'dempsystest_bjorn_step';
        codeInfo.OutputFunctions =  [codeInfo.OutputFunctions, func_UpdateContStates_Interface'];

          
         
    
      
      tmpArgs         = [];
tmpActualArgs   = [];
tmpRet          = coder.types.Argument.empty;
tmpActualReturn = RTW.DataInterface.empty;
  

  func_Terminate_Prototype = coder.types.Prototype;
  func_Terminate_Prototype.Arguments  = tmpArgs;
  func_Terminate_Prototype.Return     = tmpRet;
    func_Terminate_Prototype.HeaderFile = ['dempsystest_bjorn', '.h'];
  func_Terminate_Prototype.SourceFile = ['dempsystest_bjorn', '.c'];

  func_Terminate_Interface = RTW.FunctionInterface;
  func_Terminate_Interface.Prototype    = func_Terminate_Prototype;
  func_Terminate_Interface.ActualArgs   = tmpActualArgs;
  func_Terminate_Interface.ActualReturn = tmpActualReturn;

  time_constant = codeInfo.TimingProperties(getIndexFromTimingInternalId(codeInfo.TimingInternalIds, -1));
      func_Terminate_Interface.Timing = time_constant;
            func_Terminate_Prototype.Name = ['dempsystest_bjorn_terminate'];
      codeInfo.TerminateFunctions =  [codeInfo.TerminateFunctions, func_Terminate_Interface'];


  









  
 
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        


if ~isempty(skippedParameters)
wr.removeSkippedParameters(codeInfo.Parameters(skippedParameters));
codeInfo.Parameters(skippedParameters) = [];
end


% Handling Right-Click Builds
ss = rtwprivate('getSourceSubsystemHandle',codeInfo.GraphicalPath);
% Check if Rt-Click build, then re-map SIDs
if ~isempty(ss) && rtwprivate('rtwattic','hasSIDMap')
   codeInfo = modifyCodeInfoForSubsystemBuild(ss, codeInfo, wr);
end

wr.writeFunctionInterfaces(codeInfo);
wr.writeServerCallPoints(codeInfo);



wr.closeRepo();
  if exist('expInports', 'var')
    save codeInfo.mat codeInfo expInports;
  else
    save codeInfo.mat codeInfo;
  end
returnVal = 1;

% End Function: writeCodeInfoFcn

function data = uniquifyDataAccess(list)
  data = RTW.DataInterface.empty;
  for idx=1:numel(list)
      if ~isPresentDI(data, list(idx))
          data(end+1) = list(idx); %#ok<AGROW>
      end
  end
% End Function: uniquifyDataAccess
        
function isPresent = isPresentDI(list, di)
  isPresent = false;
  for idx = 1:numel(list)
    if isequal(list(idx), di)
      isPresent = true;
      break
    end
  end

function idx = getIndexFromTimingInternalId(internalIdVec, internalId)
 idx = find(internalIdVec == internalId);
 if (isempty(idx) || (length(idx) > 1))
    ciMsg = 'Time object cannot be empty'; 
    ciExc = MException('RTW:buildProcess:CodeInfoInternalError',ciMsg);
    throw(ciExc);
  end
% End Function: getIndexFromTimingInternalId

function checkDataGraphicalNames(ciName, rtwName)
  if (strcmp(ciName, rtwName) ~= 1)
    ciMsg = ['Name mismatch: ', ciName, ' and ', rtwName]; 
    ciExc = MException('RTW:buildProcess:CodeInfoInternalError',ciMsg);
    throw(ciExc);
  end
% End Function: checkDataGraphicalNames

function iData = getInternalDataByName(iDataVec, iDataName)
 iData = RTW.DataInterface.empty;
 for idxData = 1:numel(iDataVec)
    if strcmp(iDataName, iDataVec(idxData).GraphicalName)
       iData(end+1) = iDataVec(idxData);
       break
    end
 end
 if (isempty(iData) || (length(iData) > 1))
    ciMsg = 'Internal Data object cannot be empty'; 
    ciExc = MException('RTW:buildProcess:CodeInfoInternalError',ciMsg);
    throw(ciExc);
  end
% End Function: getInternalDataByName

function iData = getInternalDataByVariableName(iDataVec, iDataVariableName)
 iData = RTW.DataInterface.empty;
 for idxData = 1:numel(iDataVec)
    if strcmp(iDataVariableName, iDataVec(idxData).Implementation.VariableName)
       iData = iDataVec(idxData);
       break
    end
 end
 if isempty(iData)
    ciMsg = 'Internal Data object cannot be empty'; 
    ciExc = MException('RTW:buildProcess:CodeInfoInternalError',ciMsg);
    throw(ciExc);
  end
% End Function: getInternalDataByVariableName

function codeInfo = modifyCodeInfoForSubsystemBuild(ss, codeInfo, wr)
% Extract Subsystem Build Map from AtticData
    Subsystem_Build_Mapping = rtwprivate('rtwattic','getSIDMap');

% Parameters
    for i = 1:length(codeInfo.Parameters)
        sid = codeInfo.Parameters(i).SID;        

        sid = Simulink.ID.getSubsystemBuildSID(sid,ss, Subsystem_Build_Mapping);
        wr.updateSIDForSubsystemBuild("Parameter", i, sid, codeInfo.Parameters(i));
        codeInfo.Parameters(i).SID = sid;
    end

% Data Stores
     for i = 1:length(codeInfo.DataStores)
         sid = codeInfo.DataStores(i).SID;         

         sid = Simulink.ID.getSubsystemBuildSID(sid,ss, Subsystem_Build_Mapping);
         codeInfo.DataStores(i).SID = sid;
         wr.updateSIDForSubsystemBuild("DataStore", i, sid);

     end

% Inports
    for i = 1:length(codeInfo.Inports)
        sid = codeInfo.Inports(i).SID;

        sid = Simulink.ID.getSubsystemBuildSID(sid,ss, Subsystem_Build_Mapping);
        codeInfo.Inports(i).SID = sid;
        wr.updateSIDForSubsystemBuild("Inport", i, sid);

    end

% Outports
    for i = 1:length(codeInfo.Outports)
        sid = codeInfo.Outports(i).SID;

        sid = Simulink.ID.getSubsystemBuildSID(sid,ss, Subsystem_Build_Mapping);
        codeInfo.Outports(i).SID = sid;
        wr.updateSIDForSubsystemBuild("Outport", i, sid);

    end    

