classdef orientationTracker < matlab.mixin.SetGet
    properties
        modelFileName
        imuFileName
        accFileName
        model
        model_calibrated
        qTable
        aTable
        oTable
        oTable_backup
        oTable_opensim
        oTable_headingCorrection
        corrdinateDirection
        baseIMU
        sensorErrorTable
        oRefs
    end
    methods
        function self = orientationTracker(modelFileName, imuFileName, accFileName)
            import org.opensim.modeling.*
            self.imuFileName = imuFileName;
            self.accFileName = accFileName;
            self.modelFileName = modelFileName;
            self.model = Model(modelFileName);
            self.qTable = TimeSeriesTableQuaternion(imuFileName);
            self.aTable = TimeSeriesTableVec3(accFileName);
        end
        function convertQuaternionToRotations(self)
            import org.opensim.modeling.*
            % Get the table properties
            nc = self.qTable.getNumColumns();
            nt = self.qTable.getNumRows(); 
            % Get the data times
            times = self.qTable.getIndependentColumn();
            % Make a empty Matrix of Rotations
            matrix = MatrixRotation(nt, nc, Rotation());
            % Convert Quaternions to Rotations while filling the Matrix
            for i = 0 : times.size() - 1
                cnt = times.get(i);
                quatRow = self.qTable.getRowAtIndex(i);
                for j = 0 : nc - 1
                    matrix.set(i, j, Rotation(quatRow.getElt(0,j)) );
                end
            end
            % Generate a Table of rotations using the new Matrix
            oTable = TimeSeriesTableRotation(times, matrix, self.qTable.getColumnLabels());
            % Copy the MetaData to the new Matrix
            oTable = self.copyMetaData(self.qTable, oTable);
            % store oTable internally
            self.oTable = oTable;
            % Store original backup version of the orientations Table
            self.oTable_backup = oTable;
            % Disp message
            disp('Orientations Table successfully created')
        end
        function rotateOrientations2OpenSimFrame(self)
            import org.opensim.modeling.*
            % Define a negative Rotation about the world X
            R_XG = Rotation(-pi/2, CoordinateAxis(0));
            % Rotate the data
            rotTable = self.rotationOrientationTable(self.oTable, R_XG);
            % Update the stored orientation table
            self.oTable  = rotTable;
            % Store a backup of the opensim rotated table
            self.oTable_opensim = rotTable;
            % Display message
            disp('Orientations Table successfully rotated into OpenSim World')
        end
        function setBaseIMUDirection(self, baseIMU, baseHeadingDirection)
            import org.opensim.modeling.*
            
            % Determine direction from input string
            if strcmp(baseHeadingDirection,'x')
                direction = 1; axis = 0;
            elseif strcmp(baseHeadingDirection,'y')
                direction = 1; axis = 1;
            elseif strcmp(baseHeadingDirection,'z')
                direction = 1; axis = 2;
            elseif strcmp(baseHeadingDirection,'-x')
                direction = -1; axis = 0;
            elseif strcmp(baseHeadingDirection,'-y')
                direction = -1; axis = 1;
            elseif strcmp(baseHeadingDirection,'-z')
                direction = -1; axis = 2;
            else
                error( 'baseHeading direction is incorrect. Must be either x,y,z or -x,-y,-z');
            end
            
            % Generate CoordinateDirection object from the axis and
            % direction.
            self.corrdinateDirection = CoordinateDirection( CoordinateAxis(axis), direction);
            self.baseIMU = baseIMU;
            % Get the labels of the IMus from the table
            pix = self.findLabelIndex(self.oTable, self.baseIMU);
            % If no base can be found but one was provided, throw.
            if isempty(pix)
                error(['No Data column with base IMU name ' self.baseIMU ' found. Enter a different name']);
            end
            % Display message
            disp('Heading axis and direction set')
        end
        function R_HG = computeHeadingCorrection(self)
            % Base will rotate to match <base>_imu, so we must first remove the base 
            % rotation from the other IMUs to get their orientation with respect to 
            % individual model bodies and thereby compute correct offsets unbiased by the 
            % initial base orientation.
            import org.opensim.modeling.*
            % Get the labels of the IMus from the table
            pix = self.findLabelIndex(self.oTable_opensim, self.baseIMU);

            % The initial orientation of the base IMU
            base_R = self.oTable.getRowAtIndex(0).getElt(0, pix);

            % Heading direction of the base IMU in this case the pelvis_imu heading is its ZAxis    
            if self.corrdinateDirection.getAxis().isXAxis
                col = 0;
            elseif self.corrdinateDirection.getAxis().isYAxis
                col = 1;
            elseif self.corrdinateDirection.getAxis().isZAxis
                col = 2;
            end

            pelvisHeading = UnitVec3(base_R.asMat33().get(0,col),...
                                     base_R.asMat33().get(1,col),...
                                     base_R.asMat33().get(2,col));

            if self.corrdinateDirection.getDirection() < 0 
                pelvisHeading = pelvisHeading.negate();
            end

            % Get the heading of the Model base body
            s = self.model.initSystem();
            body = self.model.getBodySet.get('pelvis');
            baseRotation = body.getTransformInGround(s).R();
            groundX = UnitVec3(baseRotation.get(0,0),baseRotation.get(0,1),baseRotation.get(0,2));

            % Compute the heading angle
            u = [pelvisHeading.get(0) pelvisHeading.get(1) pelvisHeading.get(2)];
            v = [groundX.get(0) groundX.get(1) groundX.get(2)];
            CosTheta = dot(u,v)/(norm(u)*norm(v));
            angularDifference = acosd(CosTheta);
            format long g;
            %disp(['Heading correction computed to be ' num2str(angularDifference) ' degs about ground Y'])
            
            % Compute the direction of angular difference
            xproduct = cross(v,u);
            if xproduct(2) > 0
               angularCorrection = angularDifference * -1;
            else
               angularCorrection = angularDifference; 
            end
            disp(['Heading correction computed to be ' num2str(angularCorrection) ' degs about ground Y'])
            
            % Compute the Rotation Matrix about Y of the OpenSim World
            R_HG = Rotation();
            R_HG.setRotationFromAngleAboutY(deg2rad( angularCorrection ));
            
            % Rotate the table using the computed Rotation Matrix
            oTable_headingCorrection = self.rotationOrientationTable(self.oTable, R_HG);
            
            % Store the table
            self.oTable = oTable_headingCorrection;
            self.oTable_headingCorrection = oTable_headingCorrection;
            
            % Display message
            disp('Orientation Table has been rotated using the given Base Heading Direction')
        end
        function R_HG = addHeadingCorrectionFromAngle(self, angularDifference)
            import org.opensim.modeling.*
            % Compute the Rotation Matrix about Y of the OpenSim World
            R_HG = Rotation();
            R_HG.setRotationFromAngleAboutY( deg2rad( angularDifference ));
            
            % Rotate the original table using the computed Rotation Matrix
            oTable_headingCorrection = self.rotationOrientationTable(self.oTable_opensim, R_HG);
            % Store the table
            self.oTable = oTable_headingCorrection;
            self.oTable_headingCorrection = oTable_headingCorrection;
            % Display message
            disp('Orientation Table has been rotated using the given Base Heading Direction')
        end
        function calibrateModelFromOrientations(self)
            import org.opensim.modeling.*
            
            % Make a copy of the original model to work on. 
            model = self.model.clone();
            % Pose the model using the default coordinate values. 
            s0 = model.initSystem();
            s0.setTime(0);            
            model.realizePosition(s0);
            
            % Compute the relative offset between each model body and its 
            % corresponding IMU. Add an Offset frame on the model body. 
            
            for u = 0 : model.getNumBodies() - 1
                body = model.getBodySet().get(u);
                bodyName = body.getName();
                for i = 0 : self.oTable.getColumnLabels().size() - 1 
                    imuName = char(self.oTable.getColumnLabels().get(i));
                    ix = strrep(imuName,'_imu', '');
                    if strcmp(ix, bodyName)
                        
                        % Get the orientation of the model body in Ground
                        R_BG = body.getTransformInGround(s0).R();
                        
                        % Get the orientation of the IMU frame
                        R_FG = self.oTable.getRowAtIndex(0).getElt(0,i);
                        
                        % Compute the offset rotation between the body and the
                        % (rotated) IMU data. 
                        R_FB = R_BG.multiply(R_FG);
                        
                        %disp(['Computed offset for ' char(imuLabels.get(i))]);
                        %disp(['Offset is; ']);disp(R_FB);
                        %imuOffset = PhysicalOffsetFrame();

                        % disp(['Creating Model offset frame for ' char(imuLabels.get(i))]);
                        
                        % Set the point location to be the mass center of the body
                        p_FB = body.getMassCenter();
                        
                        % Add an Offset Frame to the body with the orientation of the
                        % of the IMU
                        imuOffset = PhysicalOffsetFrame(imuName,body,Transform(R_FB, p_FB));
                        brick = Brick(Vec3(0.01, 0.01, 0.002));
                        brick.setColor(Vec3(255,165,0));
                        imuOffset.attachGeometry(brick);
                        %imuOffset.attachGeometry(FrameGeometry)
                        %FrameGeometry.setColor(Vec3(255,165,0))
                        body.addComponent(imuOffset);
                        disp(['Added offset frame for '  imuName]);
                    end
                end
            end
            model.initSystem();
            % Set the calibrated model 
            self.setCalibratedModel(model)
            % Write calibrated model to file. 
            calibrated_model_name = strrep(self.modelFileName, '.osim', '_calibrated.osim');
            model.print(calibrated_model_name);
            disp(['Calibrated Model printed: ' calibrated_model_name ]);
        end
        function setCalibratedModel(self,model)
           self.model_calibrated = model;
           disp('Calibrated Model has been internally set')
        end
        function InverseKinematics(self, varargin)
            import org.opensim.modeling.*
            
            %varargin
            nRows = self.oTable.getNumRows();
            stime = self.oTable.getIndependentColumn().get(0);
            etime = self.oTable.getIndependentColumn().get(nRows-1);
            speed = 1;
            %visualize, stime, etime,speed
            visualize = varargin{1};
            if nargin == 3
                stime = varargin{2};
            elseif nargin == 4
                stime = varargin{2};
                etime = varargin{3};
            elseif nargin == 5
                stime = varargin{2};
                etime = varargin{3};
                speed = varargin{4};
            end
            
            % Make a copy of the model and orientations to perform IK with
            model = self.model_calibrated.clone();
            oTable = self.oTable.clone();
            % Instantiate a Reporter()
            ikReporter = TableReporter();
            ikReporter.setName('ik_reporter');
            % Get the IKReporter to track coordinates of the model
            coordinates = model.getCoordinateSet();
            for i = 0 : coordinates.getSize() - 1
                coord = coordinates.get(i);
                ikReporter.updInput("inputs").connect(coord.getOutput("value"), coord.getName());
                if contains(char(coord.getMotionType()), 'Translational')
                    coord.setDefaultLocked(true)
                end
            end
            % Add Reporter to model
            model.addComponent(ikReporter);

            % Instantiate the Reference objects for tracking
            mRefs = MarkersReference();
            cRefs = SimTKArrayCoordinateReference();
            
            % Check if orientation references have already been created. 
            if isempty(self.oRefs)
                oRefs = OrientationsReference(oTable);
            else
                oRefs = self.oRefs;
            end
            
            % if Visualizing
            if visualize
                model.setUseVisualizer(true);
            end

            s0 = model.initSystem();
            t0 = s0.getTime();

            % Instantiate an IK solver
            ikSolver = InverseKinematicsSolver(model, mRefs, oRefs, cRefs);
            % Set the accuracy
            accuracy = 1e-4; ikSolver.setAccuracy(accuracy);
            
            % Set the time on the solver and do the initial assembly
            if isempty(stime);sTimeIndex = 0;else;sTimeIndex = oTable.getRowIndexAfterTime(stime);end;
            if isempty(etime);eTimeIndex = 0;else;eTimeIndex = oTable.getRowIndexAfterTime(etime);end;
            if isempty(speed);speed = 1;end
            
            % Get the Vector of times
            times = oRefs.getTimes();
            
            s0.setTime(times.get(sTimeIndex));
            ikSolver.assemble(s0);
            if visualize
                model.getVisualizer().show(s0);
                model.getVisualizer().getSimbodyVisualizer().setShowSimTime(true);
            end
            
            % Construct a Table and some Arrays and Vectors for storing the
            % orientation error
            oErrors = SimTKArrayDouble( ikSolver.getNumOrientationSensorsInUse() );
            errorTable = DataTable();
            rowVec = RowVector(ikSolver.getNumOrientationSensorsInUse());
            labels = StdVectorString;
            for i = 0 : ikSolver.getNumOrientationSensorsInUse - 1
                oName = char(ikSolver.getOrientationSensorNameForIndex(i));
                labels.add(oName)
            end
            errorTable.setColumnLabels(labels);
            
            for i = sTimeIndex : speed : eTimeIndex - 1
                time = times.get(i);
                s0.setTime(time);
                ikSolver.track(s0);
                if visualize
                    model.getVisualizer().show(s0);
                else
                    disp(['Solved frame at time: ' num2str(time) ]);
                end
                
                % Compute and store the orientation error
                % Get the current Sensor Orientation
                ikSolver.computeCurrentOrientationErrors(oErrors)
                for u = 0 : ikSolver.getNumOrientationSensorsInUse - 1
                        rowVec.set(u, oErrors.getElt(u))
                end
                errorTable.appendRow(time,rowVec)
                
                % Realize to the report stage
                model.realizeReport(s0);
            end
            
            % Write coordinates to file
            report = ikReporter.getTable();
            outputFileName = 'inverseKinematics_coordinates.mot';
            STOFileAdapter.write(report,outputFileName);
            disp(['Kinematics written to file: ' outputFileName]);
            
            % Write Errors to file
            eTable = TimeSeriesTable(errorTable);
            outputFileName = 'inverseKinematics_sensors_errors.sto';
            STOFileAdapter.write(eTable,outputFileName);
            disp(['Sensor Errors written to file: ' outputFileName]); 
            % Store errors locally
            self.sensorErrorTable = eTable;
        end
        function m = getModel(self)
            m = self.model;
        end
        function setOrientationRefs(self, keepSensor)
            import org.opensim.modeling.*
            otable = self.oTable.clone();
            labels = otable.getColumnLabels();
            
            for i = 0 : labels.size() - 1
                deleteColumn = 1;
                for u = 1 : length(keepSensor)
                    if strfind(char(labels.get(i)), keepSensor{u})
                        deleteColumn = 0;
                        break
                    end
                end
                if deleteColumn
                    otable.removeColumn(labels.get(i));
                end
            end
            % Store the orientationRefs
            self.oRefs = OrientationsReference(otable);
            disp('New Orientations Reference Set');
        end
    end
    methods (Access = private, Hidden = true)
        function rotOTable = rotationOrientationTable(self, oTable, R)
            import org.opensim.modeling.*
            % Get table Properties
            nc = oTable.getNumColumns();
            nt = oTable.getNumRows();
            % Get the Data Times
            times = oTable.getIndependentColumn();
            % Make a empty Matrix of Rotations
            matrix = MatrixRotation(nt, nc, Rotation());
            % Perform the Rotations
            for i = 0 : oTable.getNumRows() - 1
                row = oTable.getRowAtIndex(i);
                for u= 0 : oTable.getNumColumns() - 1
                    matrix.set(i,u, R.multiply( row.getElt(0,u)) );
                end
            end
            % Convert Matrix into a TimesSeriesTable
            rotOTable = TimeSeriesTableRotation(times, matrix, oTable.getColumnLabels());
            % Copy the Metadata
            rotOTable = self.copyMetaData(oTable,rotOTable);
        end
        function newTable = copyMetaData(self, oldTable, newTable)
            for i = 0 : oldTable.getTableMetaDataKeys.size() - 1
                metaKey = oldTable.getTableMetaDataKeys.get(i);
                newTable.addTableMetaDataString(metaKey, oldTable.getTableMetaDataString(metaKey));
            end
        end
        function pix = findLabelIndex(self, otable, label)
            imuLabels = self.oTable.getColumnLabels();
            pix = [];
            for i = 0 : imuLabels.size() - 1
                if strcmp(label,imuLabels.get(i))
                    pix = i;
                    break
                end
            end
        end
    end
end
        
        