  
  
void ODriveSetup()  {

        Serial.println("Front Right Leg");
        for (int axis = 0; axis < 2; ++axis) {
            Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n'; 
            Serial1 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial1 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial1 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';       
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive1.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive1.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive1.run_state(axis, requested_state, false); // don't wait
        }

              
        Serial.println("Front Left Leg");
        for (int axis = 0; axis < 2; ++axis) {
            Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial2 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n'; 
            Serial2 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial2 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial2 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';             
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive2.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive2.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive2.run_state(axis, requested_state, false); // don't wait
        }   

        Serial.println("Front Undercarriage");
        for (int axis = 0; axis < 2; ++axis) {
            Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial3 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
            Serial3 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial3 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial3 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';              
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive3.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive3.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive3.run_state(axis, requested_state, false); // don't wait
        }

        Serial.println("Rear RIght Leg");
        for (int axis = 0; axis < 2; ++axis) {
            Serial4 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial4 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
            Serial4 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial4 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial4 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';              
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive4.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive4.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive4.run_state(axis, requested_state, false); // don't wait
        }  


        Serial.println("Rear Left Leg");
        for (int axis = 0; axis < 2; ++axis) {
            Serial5 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial5 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n'; 
            Serial5 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial5 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial5 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';             
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive5.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive5.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive5.run_state(axis, requested_state, false); // don't wait
        }



        Serial.println("Rear Undercarriage");
        for (int axis = 0; axis < 2; ++axis) {
            Serial6 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
            Serial6 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';  
            Serial6 << "w axis" << axis << ".controller.config.vel_gain " << 5.0f / 10000.0f << '\n'; 
            Serial6 << "w axis" << axis << ".controller.config.vel_integrator_gain " << 1.0f / 10000.0f << '\n'; 
            Serial6 << "w axis" << axis << ".controller.config.pos_gain " << 20.0f << '\n';            
        
            requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive6.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_INDEX_SEARCH ;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive6.run_state(axis, requested_state, true);      
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
            odrive6.run_state(axis, requested_state, false); // don't wait
        }


}

