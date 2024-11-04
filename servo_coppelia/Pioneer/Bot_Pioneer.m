classdef Bot_Pioneer < handle
    properties (Access = private)
        CoppeliaSim = [];
        ClientID = [];
        
        Pioneer = [];
        motorL = [];
        motorR = [];

        camera = [];

    end
    
    methods
        function obj = Bot_Pioneer()
            % Setting connection and starting simulation
            obj.CoppeliaSim = remApi('remoteApi');
            obj.CoppeliaSim.simxFinish(-1);
            disp('Connecting to robot....')
            obj.ClientID = obj.CoppeliaSim.simxStart('127.0.0.1',19997,true,true,5000,5); 

            obj.CoppeliaSim.simxStopSimulation(obj.ClientID, obj.CoppeliaSim.simx_opmode_oneshot_wait);
            obj.CoppeliaSim.simxSynchronous(obj.ClientID, true);
            obj.CoppeliaSim.simxStartSimulation(obj.ClientID, obj.CoppeliaSim.simx_opmode_blocking);

            
            if obj.ClientID < 0       % Early return     
                error(' robot not connected...')
            end

            % Setting up handlers
            [~,obj.Pioneer] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/PioneerP3DX',obj.CoppeliaSim.simx_opmode_oneshot_wait);
            [~,obj.motorL] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/PioneerP3DX/leftMotor',obj.CoppeliaSim.simx_opmode_oneshot_wait);
            [~,obj.motorR] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/PioneerP3DX/rightMotor',obj.CoppeliaSim.simx_opmode_oneshot_wait);         
            
            [~,obj.camera] = obj.CoppeliaSim.simxGetObjectHandle(obj.ClientID,'/PioneerP3DX/kinect/rgb',obj.CoppeliaSim.simx_opmode_oneshot_wait);
            
            
            obj.Initialize_Streaming(); % Initialize streaming of data
            pause(1)            
            
            obj.CoppeliaSim.simxSynchronousTrigger(obj.ClientID);

            disp(' done.')
        end


        function Initialize_Streaming(obj)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID) ~= 1  % Early returns           
                error('Connection lost during streaming initialization.');
            end

            obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID, obj.Pioneer, -1, obj.CoppeliaSim.simx_opmode_streaming);
            obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID, obj.Pioneer, -1, obj.CoppeliaSim.simx_opmode_streaming);
            obj.CoppeliaSim.simxGetVisionSensorImage2(obj.ClientID,obj.camera,0,obj.CoppeliaSim.simx_opmode_streaming);
        end
        

        %%%%%%%%%%%%%% Getters %%%%%%%%%%%%
        function p = Get_Pose (obj)
            aux = ones(2,1);

            while sum(aux)~=0

                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)~=1     % Early return          
                    error(' connection lost...')
                end

                [aux(1),position] = obj.CoppeliaSim.simxGetObjectPosition(obj.ClientID,obj.Pioneer,-1,obj.CoppeliaSim.simx_opmode_buffer);
                [aux(2),orientation] = obj.CoppeliaSim.simxGetObjectOrientation(obj.ClientID,obj.Pioneer,-1,obj.CoppeliaSim.simx_opmode_buffer);
                    
                p = double([position(1) position(2) orientation(3)]');
            end
        end
        
        function img = Get_Image (obj)
            aux = 1;
            
            while aux~=0
                if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID) ~= 1  % Early return                                  
                    error(' connection lost...')
                end
                [aux,~,img] = obj.CoppeliaSim.simxGetVisionSensorImage2(obj.ClientID,obj.camera,0,obj.CoppeliaSim.simx_opmode_buffer);
            end
        end


        %%%%%%%%%%%%%% Setters %%%%%%%%%%%%%

        function obj = Set_Joint_Velocity (obj,w)
            if obj.CoppeliaSim.simxGetConnectionId(obj.ClientID)~=1   % Early return                          
                error(' connection lost...')
            end

            obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.motorR,w(1),obj.CoppeliaSim.simx_opmode_oneshot);
            obj.CoppeliaSim.simxSetJointTargetVelocity(obj.ClientID,obj.motorL,w(2),obj.CoppeliaSim.simx_opmode_oneshot);
        end

        %%%%%%%%% Aux %%%%%%%%%%%%
        
        function s = Connection(obj)
            s = obj.CoppeliaSim.simxGetConnectionId(obj.ClientID);
        end

        function Stop_Simulation (obj)
            obj.CoppeliaSim.simxStopSimulation(obj.ClientID,obj.CoppeliaSim.simx_opmode_oneshot_wait);
            obj.Set_Joint_Velocity([0 0]');
        end

        function Simulation_Step(obj)
            obj.CoppeliaSim.simxSynchronousTrigger(obj.ClientID);
        end
    end
end