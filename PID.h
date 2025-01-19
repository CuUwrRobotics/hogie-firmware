class PID{
    public:
        PID(float constantP, float constantD, float constantI){kP = constantP; kD = constantD; kI = constantI;}
        PID(){PID(1, 0, 0);}

        float calc(float setpoint, float measurement, float currTime){
            float error = setpoint - measurement;
            float timeDiff = currTime - prevTime < 0.5 ? currTime - prevTime : 0;

            float total = calcP(error) + calcI(error, timeDiff) + calcD(error, timeDiff);
            prevTime = currTime;
            return total;
        }

    private:
        float kP, kD, kI, prevTime = 0, prevError = 0;
        float calcI(float error, float timeDiff){
            return kI * error * timeDiff;
        }
        float calcP(float error){
            return kP * error;
        }
        float calcD(float error, float timeDiff){
            float total = (error - prevError)/(timeDiff == 0? 1000 : timeDiff);
            prevError = error;
            return total;
        }
};