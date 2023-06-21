function [ctFitCoef,cqFitCoef] = calcPropDataFit(propDataFldrPath)

arguments
    propDataFldrPath (1,1) {mustBeFolder}
end

% Get the list of text files in the folder
dataFiles = dir(propDataFldrPath + "/*.txt");

% Split the files into dynamic and static datasets
stDatFile = string.empty(0,0);
dynDatFile = string.empty(0,0);
rpmVals = double.empty(0,0);
for i = 1:length(dataFiles)
    [~,fName,~] = fileparts(dataFiles(i).name);
    fNameSplit = strsplit(fName,'_');
    if strcmp(fNameSplit{3},"geom")
        continue;
    elseif strcmp(fNameSplit{3},"static")
        stDatFile(end+1) = fullfile(dataFiles(i).folder,dataFiles(i).name);
    else
        rpmVals(end+1) = double(string(fNameSplit{4}));
        dynDatFile(end+1) = fullfile(dataFiles(i).folder,dataFiles(i).name);
    end
end

% Read static data and add an average value for it in the dataset
staticDataTable = readtable(stDatFile);
J = 0; Ct = mean(staticDataTable.CT); Cq = mean(staticDataTable.CP/2/pi);

% Read dynamic data and add it to the dataset
for i = 1:length(dynDatFile)
    dynDataTable = readtable(dynDatFile(i));
    J = [J;dynDataTable.J];
    Ct = [Ct;dynDataTable.CT];
    Cq = [Cq;dynDataTable.CP/2/pi];
end

% Perform the polynomial curvefit
ctFitCoef = coeffvalues(fit(J,Ct,fittype('poly2')));
cqFitCoef = coeffvalues(fit(J,Cq,fittype('poly2')));
end