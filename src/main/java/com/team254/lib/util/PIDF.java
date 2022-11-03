package com.team254.lib.util;
import edu.wpi.first.wpilibj.Timer;

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
    public PIDF(double kP, double kI, double kD, double steadyStateQualRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kSteadyStateQualRange = steadyStateQualRange;
        this.prevTime = Timer.getFPGATimestamp();
        this.prevError = 0.0;
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

    void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    double update(double curRefPoint) {
        double curTime = Timer.getFPGATimestamp();
        double deltaTime = curTime - this.prevTime;
        // updating the reference point, and calculating new error
        this.refPoint = curRefPoint;
        this.curError = this.setPoint - this.refPoint;

        double proportionalTerm = this.kP * this.curError;

        // calculating dervivativeTerm / dampening term
        double derivativeTerm = this.kD * (this.curError - this.prevError) / (deltaTime);

        // integral term will be 0 unless in steady state

        // checking for steady state
        if (Math.abs(this.curError) < this.kSteadyStateQualRange) {
            if (this.steadyStateErrorStartTime == -1) {
                this.steadyStateErrorStartTime = curTime;
            }
            // note that at each time, a new rectangle is being added
            // using overEstimation for Integrals
            //perhaps better practice to use a variable total_error and multiply by kI for integral term
            this.integralTerm += kI * (deltaTime) * (this.curError);
        }

        // update the time and errors
        this.prevTime = curTime;
        this.prevError = this.curError;

        double output = proportionalTerm + derivativeTerm + integralTerm;

        return output;
    }

    
}