/**
 * Class represents SCARA robotic arm.
 * 
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import java.awt.Color;
import java.util.*;

public class Arm{

    // fixed arm parameters
    private int xm1;  // coordinates of the motor(measured in pixels of the picture)
    private int ym1;
    private int xm2;
    private int ym2;
    private double r;  // length of the upper/fore arm
    // parameters of servo motors - linear function pwm(angle)
    // each of two motors has unique function which should be measured
    // linear function cam be described by two points

    // current state of the arm
    private double theta1; // angle of the upper arm
    private double theta2;
    private int pwm1;
    private int pwm2;

    private double xj1;     // positions of the joints
    private double yj1; 
    private double xj2;
    private double yj2; 
    private double xt;     // position of the tool
    private double yt;
    private boolean valid_state; // is state of the arm physically possible?

    /**
     * Constructor for objects of class Arm
     */
    public Arm(){
        xm1 = 290; // set motor coordinates
        ym1 = 372;
        xm2 = 379;
        ym2 = 374;
        r = 156.0;
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
    }

    // draws arm on the canvas
    public void draw(){
        // draw arm
        int height = UI.getCanvasHeight();
        int width = UI.getCanvasWidth();
        // calculate joint positions
        xj1 = xm1 + r*Math.cos(theta1);
        yj1 = ym1 + r*Math.sin(theta1);
        xj2 = xm2 + r*Math.cos(theta2);
        yj2 = ym2 + r*Math.sin(theta2);

        //draw motors and write angles
        int mr = 20;
        UI.setLineWidth(5);
        UI.setColor(Color.BLUE);
        UI.drawOval(xm1-mr/2,ym1-mr/2,mr,mr);
        UI.drawOval(xm2-mr/2,ym2-mr/2,mr,mr);
        // write parameters of first motor
        String out_str=String.format("t1=%3.1f",theta1*180/Math.PI);    
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+2*mr);
        out_str=String.format("xm1=%d",xm1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+3*mr);
        out_str=String.format("ym1=%d",ym1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+4*mr);
        out_str=String.format("pwm1=%d",pwm1);
        UI.drawString(out_str, xm1-2*mr,ym1-mr/2+5*mr);
        // ditto for second motor                
        out_str = String.format("t2=%3.1f",theta2*180/Math.PI);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+2*mr);
        out_str=String.format("xm2=%d",xm2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+3*mr);
        out_str=String.format("ym2=%d",ym2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+4*mr);
        out_str=String.format("pwm2=%d",pwm2);
        UI.drawString(out_str, xm2+2*mr,ym2-mr/2+5*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);

        // it can b euncommented later when
        // kinematic equations are derived
        if ( valid_state) {
            // draw upper arms
            UI.setColor(Color.GREEN);
            UI.drawLine(xm1,ym1,xj1,yj1);
            UI.drawLine(xm2,ym2,xj2,yj2);
            //draw forearms
            UI.drawLine(xj1,yj1,xt,yt);
            UI.drawLine(xj2,yj2,xt,yt);
            // draw tool
            double rt = 20;
            UI.drawOval(xt-rt/2,yt-rt/2,rt,rt);
        }

    }

    // calculate tool position from motor angles 
    // updates variable in the class
    public void directKinematic(){

        // midpoint between joints
        double  Ax = ((xj1+xj2)/2);
        double  Ay = ((yj1+yj2)/2);
        // distance between joints
        double d = Math.sqrt((Math.pow(xj2-xj1, 2))+(Math.pow(yj2-yj1, 2)));
        if (d<2*r){
            valid_state = true;
            // half distance between tool positions
            double  h = (Math.sqrt((Math.pow(r, 2))-Math.pow(Ax-xj1, 2)-Math.pow(Ay-yj1, 2)));
            double alpha = (Math.atan((yj1-yj2)/(xj2-xj1)));
            // tool position
            double xt = Ax + (h*Math.cos((Math.PI/2) - alpha));
            double yt = Ax + (h*Math.sin((Math.PI/2) - alpha));
            double xt2 = Ax - (h*Math.cos((Math.PI/2) - alpha));
            double yt2 = Ay - (h*Math.sin((Math.PI/2) - alpha));
        } else {
            valid_state = false;
        }

    }

    // motor angles from tool position
    // updetes variables of the class
    public void inverseKinematic(double xt_new,double yt_new){

        valid_state = true;
        xt = xt_new;
        yt = yt_new;
        valid_state = true;
        double dx1 = xm1 - xt; 
        double dy1 = ym1 - yt;
        // distance between pen and motor
        double d1 = Math.sqrt((Math.pow(dx1, 2))+(Math.pow(dy1, 2)));
        if (d1>2*r){
            //UI.println("Arm 1 - can not reach");
            valid_state = false;
            return;
        }

        double l1 = d1/2;
        double h1 = Math.sqrt(r*r - d1*d1/4);
        // elbows positions
        double beta1 = Math.atan2((yt-ym1), (xt-xm1));
        double alpha1 = (((Math.PI)/2.0)-((Math.PI) -beta1));
        xj1 = (xm1+((xt-xm1)/2.0))+(h1*Math.cos(alpha1));
        yj1 = (ym1+((yt-ym1)/2.0))+(h1*Math.sin(alpha1));

        theta1 = Math.atan2((yj1-ym1), (xj1-xm1));
        if ((theta1>0)||(theta1<-Math.PI)){
            valid_state = false;
            //UI.println("Ange 1 -invalid");
            return;
        }

        //double theta12 = atan2(yj12 - ym1,xj12-xm1);
        double dx2 = xm2 - xt; 
        double dy2 = ym2 - yt;
        double d2 = Math.sqrt((Math.pow(dx2, 2))+(Math.pow(dy2, 2)));;
        if (d2>2*r){
            // UI.println("Arm 2 - can not reach");
            valid_state = false;
            return;
        }

        double l2 = d2/2;

        double h2 = Math.sqrt(r*r - d2*d2/4);
        // elbows positions
        double beta2 = Math.atan2((yt-ym2), (xt-xm2));
        double alpha2 = (((Math.PI)/2.0)-((Math.PI)-beta2));
        xj2 = (xm2+((xt-xm2)/2.0))-(h2*Math.cos(alpha2));
        yj2 = (ym2+((yt-ym2)/2.0))-(h2*Math.sin(alpha2));
        // motor angles for both 1st elbow positions
        theta2 = Math.atan2((yj2-ym2), (xj2-xm2));;
        if ((theta2>0)||(theta2<-Math.PI)){
            valid_state = false;
            //UI.println("Angle 2 -invalid");
            return;
        }
        
        if(yt>=(yj2+((yj1-yj2)/2.0)-5)){
            valid_state = false;
            return;
        }
        //UI.printf("xt:%3.1f, yt:%3.1f\n",xt,yt);
        //UI.printf("theta1:%3.1f, theta2:%3.1f\n",theta1*180/Math.PI,theta2*180/Math.PI);
        return;
    }
    

    // returns angle of motor 1
    public double get_theta1(){
        return theta1;
    }
    // returns angle of motor 2
    public double get_theta2(){
        return theta2;
    }
    // sets angle of the motors
    public void set_angles(double t1, double t2){
        theta1 = t1;
        theta2 = t2;
    }
    
    public void set_pwms(){ 
        int pwm1 = (int)(-10.537*theta1 + 563.709);
        int pwm2 =(int)(-10.616*theta2 + 704.607);
    }

    // returns motor control signal
    // for motor to be in position(angle) theta1
    // linear intepolation
    public int get_pwm1(){
        return pwm1;
    }
    // ditto for motor 2
    public int get_pwm2(){
        return pwm2;
    }

}
