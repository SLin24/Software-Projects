public class PIDF {
    final double kP;
    final double kI;
    final double kD;
    final double kSteadyStateQualRange;
    double setPoint;
    double refPoint;
    double prevError = 0;
    double curError;
    double prevTime;
    double steadyStateErrorStartTime = -1;
    double integralTerm = 0;
    double kF = 0;
    

    // initializes gains, and defines a setpoint (assuming setPoint is constant)
    // note that steadyStateQualRange should be a positive value
    public PIDF(double kP, double kI, double kD, double setPoint, double refPoint, double curTime, double steadyStateQualRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kSteadyStateQualRange = steadyStateQualRange;
        this.setPoint = setPoint;
        this.refPoint = refPoint;
        this.prevError = setPoint - refPoint;
        this.prevTime = curTime;
    }

    void setArbitraryFF(double val) {
        this.kF = val;
    }
    // other FF definitions like setPoint and reference based need specific formulas. (Better to define in a child class)

    double getKP() {
        return this.kP;
    }

    double getKI() {
        return this.kI;
    }

    double getKD() {
        return this.kD;
    }

    double getkF() {
        return this.kF;
    }

    double update(double newRefPoint, double curTime) {
        double output = 0;
        double deltaTime = curTime - this.prevTime;
        // updating the reference point, and calculating new error
        this.refPoint = newRefPoint;
        this.curError = this.setPoint - this.refPoint;

        double proportionalTerm = this.kP * this.curError;

        // calculating dervivativeTerm / dampening term
        double derivativeTerm = this.kD * (this.curError - this.prevError) / (deltaTime);

        // integral term will be 0 unless in steady state

        // checking for steady state
        if (Math.abs(this.prevError) < this.kSteadyStateQualRange) {
            if (this.steadyStateErrorStartTime == -1) {
                this.steadyStateErrorStartTime = this.prevTime;
            }
            // note that at each time, a new rectangle is being added
            // will not be extremely precise, but still relatively accurate if update runs frequently enough, (also because error changes at a slower rate)
            // using underEstimation for Integrals, to prevent oscillation / overshooting
            this.integralTerm += kI * (deltaTime) * (this.curError);
        }

        // update the time and errors
        this.prevTime = curTime;
        this.prevError = this.curError;

        output = proportionalTerm + derivativeTerm + integralTerm;

        return output;
    }

    
}