import processing.net.*;
import gab.opencv.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;

class Vision
{
    int currentState;
    int DOISize = floor(75*0.75);
    float x1,y1,angle;
    color backColor;
    color foreColor;
    PVector centroidG, cent1, cent2;
    PImage iRobot;
    OpenCV opencvC;
    PApplet app;

    Vision(PApplet app, color backColor, color foreColor)
    {
        this.backColor    = backColor;
        this.foreColor    = foreColor;
        this.currentState = 0;
        this.app          = app;
    }

    Vision(PApplet app, color backColor)
    {
        this.backColor    = backColor;
        this.currentState = 0;
        this.app          = app;
        this.angle        = 0;
    }

    // Function to find the pixels that have the color specified
    PImage detectColor(PImage mImage,color searchColor)
    {
    mImage.loadPixels();
    this.opencvC.loadImage(mImage); //,mImage.width, mImage.height);
    this.opencvC.useColor(HSB);
    this.opencvC.setGray(this.opencvC.getH().clone());
    int hue = int(map(hue(searchColor), 0, 255, 0, 180));
    this.opencvC.inRange(hue-1, hue+1);
    this.opencvC.dilate();
    this.opencvC.dilate();
    this.opencvC.erode();this.opencvC.erode();this.opencvC.erode();
    this.opencvC.invert();
    this.opencvC.erode();this.opencvC.erode();this.opencvC.erode();this.opencvC.erode();
    this.opencvC.invert();
    return(this.opencvC.getSnapshot());  //<>//
    }
    
    // Function to find the instersection of pixels between two images
    // Inputs: images to be intersected
    // Output: the resulting image after intersection
    PImage intersection(PImage im1, PImage im2)
    {
        PImage temp = createImage(im1.width, im1.height, RGB);
        temp.loadPixels();
        for (int loc=0; loc<temp.pixels.length; loc++)
        {
                int pixelColor1 = im1.pixels[loc];
                int pixelColor2 = im2.pixels[loc];
                colorMode(RGB);
                if ((pixelColor1==color(255))&&(pixelColor2==color(255)))
                {
                temp.pixels[loc]=color(255);
                }
                else
                { 
                temp.pixels[loc]=color(0);
                
                }
                }
            temp.updatePixels();
            return(temp);
    }

    // Computes the centroid as a medium point from other coordinates
    PVector computeCentroid(PShape ss)
    {
    // Computes the centroid of the first circle
        float top,bottom,left,right;
        top=ss.height;
        bottom = 0;
        left = ss.width;
        right = 0;
        PVector tempVector= new PVector(0,0);
        for (int j=0;j<ss.getVertexCount();j++)
        {
        tempVector=ss.getVertex(j);
        if (tempVector.x<left)
        {
            left=tempVector.x;
        }
        if (tempVector.x>right)
        {
            right=tempVector.x;
        }
        if (tempVector.y<top)
        {
            top=tempVector.y;
        }
        if (tempVector.y>bottom)
        {
            bottom=tempVector.y;
        }    
        }
        float c11    = left + (right-left) / 2;
        float c12    = (top + (bottom-top) / 2);
        tempVector.x = c11;
        tempVector.y = c12;
        
        return (tempVector);
    }


    void findCircles(PImage imgG, boolean zona)
    {
        PShape tempShape;
        ArrayList<Contour> contours;
        int count=0;
        PVector tempVe=null;
        PVector[] centroidsArray=new PVector[6];
        PShape[] shapesArray = new PShape[6];
        PVector[] vertex3 = new PVector[3];
        this.opencvC.loadImage(imgG);
        this.opencvC.erode();
        contours = this.opencvC.findContours();
        for (Contour contour : contours)
        {
        tempShape = createShape();
        tempShape.beginShape();
        for (PVector point : contour.getPolygonApproximation().getPoints())
        {
            tempShape.vertex(point.x, point.y);
        }
        tempShape.endShape(CLOSE);
        if (count<6){
        shapesArray[count]=tempShape;
        centroidsArray[count] = computeCentroid(shapesArray[count]);
        count=count+1; 
        }
        println(count);
        }
        if (count == 6)
        {
        int cta=0;
        for (int k=0;k<5;k++)
        {
            for (int m=k+1;m<6;m++)
            {
                if (compareShapes(centroidsArray[k],centroidsArray[m])==true)
                {
                vertex3[cta] = centroidsArray[k];
                cta=cta+1;
                }
            }
        }
        // Is the common vertex 1?
        PVector  u = new PVector(vertex3[0].x,vertex3[0].y);
        u.sub(vertex3[1]);
        PVector  v = new PVector(vertex3[2].x,vertex3[2].y);
        v.sub(vertex3[1]);
        float pp = u.dot(v)/(u.mag()*v.mag());
        // A right angle is considered between 85 deg (1.48 rad) and 95 deg (1.65 rad)
        if ((pp<cos(1.48))&&(pp>cos(1.65)))  
        {
            u = PVector.sub(vertex3[0],vertex3[2]);
            u.mult(0.5);
            u.add(vertex3[2]);
            tempVe = u;
            this.angle=PVector.sub(vertex3[1],u).heading()*180/PI;       
        }
        else
        {
            // Is the common vertex 2?
            u = new PVector(vertex3[1].x,vertex3[1].y);
            u.sub(vertex3[2]);
            v = new PVector(vertex3[0].x,vertex3[0].y);
            v.sub(vertex3[2]);
            pp = u.dot(v)/(u.mag()*v.mag());
            // A right angle is considered between 80 and 90 deg (80 deg = 1.4 rad)
            if ((pp<cos(1.48))&&(pp>cos(1.65)))  // If the right angle is between 1 and 2
            {
            u = PVector.sub(vertex3[1],vertex3[0]);
            u.mult(0.5);
            u.add(vertex3[0]);
            tempVe = u;
            this.angle=PVector.sub(vertex3[2],u).heading()*180/PI;
            }      
            else
            {
            // The common vertex is 0
            u = PVector.sub(vertex3[1],vertex3[2]);
            u.mult(0.5);
            u.add(vertex3[2]);
            tempVe = u; 
            this.angle=PVector.sub(vertex3[0],u).heading()*180/PI;
            }
        }
        this.angle=180 - this.angle-45; 
        if (this.angle<0){
            this.angle = this.angle+360;
        }
        if (zona == true)
        {
            this.centroidG.x = this.centroidG.x-(2*DOISize)+tempVe.x;
            this.centroidG.y = this.centroidG.y-(2*DOISize)+tempVe.y;
        }
        else
        {
            this.centroidG = tempVe;
        }
        }
    }

    void findBall(PImage pImg, boolean zona)
    {
        PShape sss;
        PVector tempV;
        ArrayList<Contour> contours;
        
        pImg.loadPixels();
        this.opencvC.loadImage(pImg);
        this.opencvC.useColor(HSB);
        this.opencvC.setGray(this.opencvC.getH().clone());
        int hue = int(map(hue(ball_color), 0, 255, 0, 180));
        this.opencvC.inRange(hue-3, hue+3);
        this.opencvC.erode();this.opencvC.erode();this.opencvC.erode();
        contours = this.opencvC.findContours();
        sss = createShape();
        sss.beginShape(POINTS);
        for (Contour contour : contours)
        {
        for (PVector point : contour.getPolygonApproximation().getPoints())
        {
            sss.vertex(point.x, point.y);
        }
        }
        sss.endShape(CLOSE);
        tempV = computeCentroid(sss);
        if (zona == true)
        { 
        this.centroidG.x = this.centroidG.x-2*DOISize+tempV.x;
        this.centroidG.y = this.centroidG.y-2*DOISize+tempV.y;
        }
        else
        {
        this.centroidG = tempV;
        }
    }
    
    void update(int element)
    {
        PImage h;
        PImage myimage,image1,image2;
        int ballNumber= 2;
        int robotNumber = 1;
        
        switch(this.currentState)
        {
        case 0:
                myimage = get();
                opencvC = new OpenCV(app,myimage);
                if (element == robotNumber)
                {
                image1 = detectColor(myimage,this.backColor);
                image2 = detectColor(myimage,this.foreColor);
                h = intersection(image1,image2);
                findCircles(h,false);
                this.iRobot = get(floor(this.centroidG.x-2*DOISize),floor(this.centroidG.y-2*DOISize),floor(4*DOISize),floor(4*DOISize));
                }
                if (element == ballNumber)
                {
                findBall(myimage,false);
                this.iRobot = get(floor(this.centroidG.x-2*DOISize),floor(this.centroidG.y-2*DOISize),floor(4*DOISize),floor(4*DOISize));
                }            
                break;
            case 1:
                opencvC = new OpenCV(app,iRobot);
                break;
            case 2:
                if (element == robotNumber)
                {        
                this.iRobot = get(floor(this.centroidG.x-2*DOISize),floor(this.centroidG.y-2*DOISize),floor(4*DOISize),floor(4*DOISize));
                image1 = detectColor(this.iRobot,this.backColor);
                image2 = detectColor(this.iRobot,this.foreColor);
                h = intersection(image1,image2);
                findCircles(h,true);
                }
                if (element == ballNumber)
                {
                this.iRobot = get(floor(this.centroidG.x-2*DOISize),floor(this.centroidG.y-2*DOISize),floor(4*DOISize),floor(4*DOISize));
                findBall(this.iRobot,true);
                }             
                break;
        }
        this.currentState = this.currentState + 1;
        if (this.currentState == 3)
        {
        this.currentState=2;
        }
    }
    
    boolean compareShapes(PVector Pv1, PVector Pv2)
    {
        if ((abs(Pv1.x-Pv2.x)<3) && (abs(Pv1.y-Pv2.y)<3))
        {
            return(true);
        }
        else
        {
            return(false);
        }
    }
  
}
