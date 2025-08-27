class PIDController {
private:
    float kp;
    float ki;
    float kd;
    float minOutput;
    float maxOutput;
    float integral;
    float previousError;
    bool firstUpdate;
    
    // Helper function to clamp values
    float clamp(float value, float min, float max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    } 

public:
    PIDController(float p, float i, float d, float minOut = -100.0, float maxOut = 100.0) {
        kp = p;
        ki = i;
        kd = d;
        minOutput = minOut;
        maxOutput = maxOut;
        integral = 0.0;
        previousError = 0.0;
        firstUpdate = true;
    }

    float compute(float error, float dt) {
        if (dt <= 0) {
            return 0.0;
        }

        // P term
        float pTerm = kp * error;
        
        // I term
        integral = clamp(integral + error * dt, -100.0, 100.0);
        float iTerm = ki * integral;
        
        // D term
        float dTerm;
        if (firstUpdate) {
            dTerm = 0.0;
            firstUpdate = false;
        } else {
            dTerm = kd * (error - previousError) / dt;
        }
        
        previousError = error;
        
        // Calculate and clamp final output
        float output = clamp(pTerm + iTerm + dTerm, minOutput, maxOutput);
        return output;
    }

    // Reset the controller
    void reset() {
        integral = 0.0;
        previousError = 0.0;
        firstUpdate = true;
    }
};
int readLDR(int pin) {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return sum / 5;
}