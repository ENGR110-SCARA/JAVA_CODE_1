
/* Code for Assignment ?? 
 * Name:
 * Usercode:
 * ID:
 */

import ecs100.*;
import java.util.*;
import java.io.*;
import java.awt.*;

/** <description of class Main>
 */
public class Main{

    private Arm arm;
    private Drawing drawing;
    private ToolPath tool_path;
    // state of the GUI
    private int state; // 0 - nothing
    // 1 - inverse point kinematics - point
    // 2 - enter path. Each click adds point  
    // 3 - enter path pause. Click does not add the point to the path

    /**      */
    public Main(){
        UI.initialise();
        UI.addButton("xy to angles", this::inverse);
        UI.addButton("Enter path XY", this::enter_path_xy);
        UI.addButton("Check range of tool", this::check_range_xy);
        UI.addButton("Draw Line (CORE)", this::drawLine);
        UI.addButton("Draw rectangle (CORE)", this::drawRect);
        UI.addButton("Draw oval (CORE)", this::drawOval);
        UI.addButton("File transfer to pi (COMPLETION)", this::transferFile);
        UI.addButton("Save path XY", this::save_xy);
        UI.addButton("Load path XY", this::load_xy);
        UI.addButton("Save path Ang", this::save_ang);
        UI.addButton("Load path Ang:Play", this::load_ang);

        // UI.addButton("Quit", UI::quit);
        UI.setMouseMotionListener(this::doMouse);
        UI.setKeyListener(this::doKeys);

        //ServerSocket serverSocket = new ServerSocket(22); 
        this.arm = new Arm();
        this.drawing = new Drawing();
        this.run();
        arm.draw();
    }

    public void doKeys(String action){
        UI.printf("Key :%s \n", action);
        if (action.equals("b")) {
            // break - stop entering the lines
            state = 3;
            //

        }

    }

    public void doMouse(String action, double x, double y) {
        //UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
        //   action,state,x,y);
        UI.clearGraphics();
        String out_str=String.format("%3.1f %3.1f",x,y);
        UI.drawString(out_str, x+10,y+10);
        // 
        if ((state == 1)&&(action.equals("clicked"))){
            // draw as 

            arm.inverseKinematic(x,y);
            arm.draw();
            return;
        }

        if ( ((state == 2)||(state == 3))&&action.equals("moved") ){
            // draw arm and path
            arm.inverseKinematic(x,y);
            arm.draw();

            // draw segment from last entered point to current mouse position
            if ((state == 2)&&(drawing.get_path_size()>0)){
                PointXY lp = new PointXY();
                lp = drawing.get_path_last_point();
                //if (lp.get_pen()){
                UI.setColor(Color.GRAY);
                UI.drawLine(lp.get_x(),lp.get_y(),x,y);
                // }
            }
            drawing.draw();
        }

        // add point
        if (   (state == 2) &&(action.equals("dragged"))){
            // add point(pen down) and draw
            UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,true); // add point with pen down

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
        }

        if (   (state == 3) &&(action.equals("clicked"))){
            // add point and draw
            //UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,false); // add point wit pen up

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
            state = 2;
        }

    }
    
    public void drawLine(){
        int Xstart = 270;
        int Xfinish = 400;
        int Y = 150;

        //plot line adding points to list
        drawing.add_point_to_path(Xstart, Y, true);
        drawing.add_point_to_path(Xfinish, Y, false);
    }

    public void drawRect(){ //We will need to scale these to ensure that the sides are exactly 40mm
        int Xleft = 275 + 50;
        int Xright = 350+ 50;
        int Ytop = 110;
        int Ybottom = 185;
        
        drawing.add_point_to_path(Xleft, Ytop, true);
        drawing.add_point_to_path(Xright, Ytop, true);
        drawing.add_point_to_path(Xright, Ybottom, true);
        drawing.add_point_to_path(Xleft, Ybottom, true);
        drawing.add_point_to_path(Xleft, Ytop, true);
    }

    public void drawOval(){
        int radius = 37;    //half diameter
        double centreX = 380;
        double centreY = 140;

        double X;
        double Y;

        //plot line adding points to list
        for(int i = 0; i<=366; i++){
            X = centreX + radius*Math.cos((((double)i/180)*Math.PI));
            Y = centreY + radius*Math.sin((((double)i/180)*Math.PI));
            drawing.add_point_to_path(X, Y, true);
        }
    }
    
    public void transferFile(){
        //Require PSCP in root folder
        //Can be downloaded at: http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html
        String fname = UIFileChooser.open();
        try {
            Process p = Runtime.getRuntime().exec("pscp -l pi -pw pi "+fname+" pi@10.140.194.193:/home/pi/Arm/");
            UI.println("Sent");
        }catch(Exception e){
            UI.println(e);
        }
    }
    
    public void save_xy(){
        state = 0;
        String fname = UIFileChooser.save();
        drawing.save_path(fname);
    }

    public void enter_path_xy(){
        state = 2;
    }

    public void check_range_xy(){
        for(int x=0; x<640; x++){
            for(int y=0; y<480; y++){
                // draw arm and path
                if(arm.testToolRange(x,y)) UI.setColor(Color.green);
                else UI.setColor(Color.red);
                UI.drawLine(x,y,x,y);
                //drawing.draw();
            }
        }
    }

    public void inverse(){
        state = 1;
        arm.draw();
    }

    public void load_xy(){
        state = 0;
        String fname = UIFileChooser.open();
        drawing.load_path(fname);
        drawing.draw();

        arm.draw();
    }

    // save angles into the file
    public void save_ang(){
        String fname = UIFileChooser.open();
        tool_path.convert_drawing_to_angles(drawing,arm,fname);
    }

    public void load_ang(){
    }
    public void run() {
        while(true) {
            arm.draw();
            UI.sleep(20);
        }
    }

    public static void main(String[] args){
        Main obj = new Main();
    }    

}
