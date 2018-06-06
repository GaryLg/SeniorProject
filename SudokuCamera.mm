//
//  SudokuCamera.mm
//  sudokuSolver
//
//  Created by Gary Liang on 5/12/18.
//  Copyright © 2018 Gary Liang. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <set>
#include <vector>
#include <unordered_set>
#include <string>
#include <map>

#import <opencv2/opencv.hpp>
#import <opencv2/videoio/cap_ios.h>
#import <opencv2/imgcodecs/ios.h>
#import <TesseractOCR/TesseractOCR.h>
#include <TesseractOCR/baseapi.h>
#include "leptonica/src/allheaders.h"
#include "SudokuCamera.h"
#include "Sudoku.mm"

using namespace cv;
using namespace std;


@interface SudokuCamera () <CvVideoCameraDelegate>
@end

@implementation SudokuCamera
{
    UIViewController<SudokuCameraDelegate> * delegate;
    UIImageView * imageView;
    CvVideoCamera * videoCamera;
    
    
    int mode;
    int ratioSize       ;
    int kernelSize  ;
    int threshold ;
    int fontSize;
    
}
    UIImage *imageOrientation;
    //G8Tesseract *tessMachine = [[G8Tesseract alloc] initWithLanguage:@"eng"];
    //tesseract::TessBaseAPI tess;
    Mat frame, grayedFrame,grayedFrameBlurred,edgeDetection,color_edges; //raw_frame
    Mat frameHold;
    vector<vector<unsigned int>> hardcodeDigits(9,vector<unsigned int>(9));
    float phi;
    float degrees;
    double alpha;
    double beta;
    double x;
    double y;
    int columnSize;
    int rowSize;
    int boundaryLine;
    int sudokuGridSize;
    int numberOfLines;
    int fontSize;

/*
 * ViewController will access this function which will call the Camerafrom the AVFoundation, then use
 * the OpenCV camera wrapper class to enable frame editing later on.
 */
- (id)initWithController:(UIViewController<SudokuCameraDelegate>*)c andImageView:(UIImageView*)iv
{

    ratioSize    = 90;
    kernelSize  = 3;
    threshold = 30;
    boundaryLine = 2000;
    sudokuGridSize = 9;
    numberOfLines = 2;
    fontSize = 1;
    
    delegate = c;
    imageView = iv;
    videoCamera = [[CvVideoCamera alloc] initWithParentView:imageView];
    videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack; // Use the back camera
    videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait; // Ensure proper orientation
    videoCamera.rotateVideo = YES; // Ensure proper orientation
    videoCamera.defaultFPS = 30; // How often 'processImage' is called, adjust based on the amount/complexity of images
    videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPresetPhoto;
    videoCamera.delegate = self;

    
    BOOL loaded = videoCamera.captureSessionLoaded;
    NSLog(loaded ? @"Yes" : @"No");
    
    //tess.Init(NULL, "eng", tesseract::OEM_TESSERACT_CUBE_COMBINED);
    
    hardcodeDigits[0][0] = 5;
    hardcodeDigits[0][5] = 1;
    hardcodeDigits[1][0] = 8;
    hardcodeDigits[1][2] = 7;
    hardcodeDigits[1][3] = 3;
    hardcodeDigits[1][7] = 2;
    hardcodeDigits[1][8] = 1;
    hardcodeDigits[2][0] = 1;
    hardcodeDigits[2][3] = 6;
    hardcodeDigits[2][4] = 8;
    hardcodeDigits[2][6] = 4;
    hardcodeDigits[3][4] = 3;
    hardcodeDigits[3][7] = 4;
    hardcodeDigits[4][1] = 6;
    hardcodeDigits[4][7] = 8;
    hardcodeDigits[5][1] = 3;
    hardcodeDigits[5][4] = 5;
    hardcodeDigits[6][2] = 8;
    hardcodeDigits[6][4] = 9;
    hardcodeDigits[6][5] = 3;
    hardcodeDigits[6][8] = 2;
    hardcodeDigits[7][0] = 6;
    hardcodeDigits[7][1] = 1;
    hardcodeDigits[7][5] = 7;
    hardcodeDigits[7][6] = 8;
    hardcodeDigits[7][8] = 4;
    hardcodeDigits[8][3] = 8;
    hardcodeDigits[8][8] = 5;
    return self;
}


/*
 * This function will execute everytime a frame is ran by the camera. It is done frame by frame.
 * Function uses OpenCV functions such as Canny and HoughLines to recognize shapes.  In this case,
 *  we will be using these functions to specifically identify a sudoku board. In the end, the sudoku
 * board will create a sudokuSolver class to solve the sudoku board.
 */
- (void)processImage:(cv::Mat &)img {
      cvtColor(img, frame, CV_BGRA2BGR);
        columnSize = frame.cols;
        rowSize = frame.rows;

        cvtColor(frame, grayedFrame, CV_BGR2GRAY);
        blur( grayedFrame, grayedFrameBlurred, cv::Size(3,3));
            
        Canny( grayedFrameBlurred, edgeDetection, threshold, ratioSize, kernelSize );

        // OpenCV HoughLines allows us to detect lines from the sudoku board edges.
        vector<Vec2f> lineDetection;
        HoughLines(edgeDetection, lineDetection, numberOfLines, CV_PI/180, 300, 0, 0 );
                
        vector<RecognitionLines> linePosition(lineDetection.size());
        for(int i = 0; i < lineDetection.size(); i++ ) {
            phi = lineDetection[i][0];
            degrees = lineDetection[i][1];
            alpha = cos(degrees);
            beta = sin(degrees);
            
            x = alpha * phi;
            y = beta * phi;
            
            linePosition[i].criticalPoint1.x = cvRound(x + boundaryLine*(-beta));
            linePosition[i].criticalPoint1.y = cvRound(y + boundaryLine*(alpha));
            linePosition[i].criticalPoint2.x = cvRound(x - boundaryLine*(-beta));
            linePosition[i].criticalPoint2.y = cvRound(y - boundaryLine*(alpha));
        }
                
            vector<pair<double,RecognitionLines>> horizontalLines;
            vector<pair<double,RecognitionLines>> verticalLines;
    
    
            for(int i = 0; i < linePosition.size(); i++ )
                if(lineDetection[i][1]< CV_PI/20 or lineDetection[i][1] > CV_PI-CV_PI/20) // Aligns Vertically
                    verticalLines.push_back(make_pair(lineDetection[i][0],linePosition[i]));
                else if(abs(lineDetection[i][1]-CV_PI/2)<CV_PI/20)                  // Aligns Horiztonally
                    horizontalLines.push_back(make_pair(lineDetection[i][0],linePosition[i]));

    
                sort(verticalLines.begin(), verticalLines.end(), [](const pair<double, RecognitionLines> &leftSide, const pair<double,RecognitionLines> &rightSide) {return leftSide.first < rightSide.first;});
                sort(horizontalLines.begin(), horizontalLines.end(), [](const pair<double, RecognitionLines> &leftSide, const pair<double,RecognitionLines> &rightSide) {return leftSide.first < rightSide.first;});
    
                for(int i = 0; i < verticalLines.size(); i++)
                    verticalLines[i].second.ContainerID = i;
                for(int i= 0; i < horizontalLines.size(); i++)
                    horizontalLines[i].second.ContainerID = i;
    
    
                for(auto& vertical: verticalLines) {
                    for(auto& horizontal: horizontalLines) {
                        Point2f intersectionPoints;
                        if(intersection(vertical.second.criticalPoint1, vertical.second.criticalPoint2,
                                        horizontal.second.criticalPoint1, horizontal.second.criticalPoint2, intersectionPoints)) {
                            if(intersectionPoints.x >= 0 and intersectionPoints.x < columnSize and intersectionPoints.y >= 0 and intersectionPoints.y < rowSize) {
                                vertical.second.intersections.insert(make_pair(intersectionPoints.y, horizontal.second.ContainerID));
                                horizontal.second.intersections.insert(make_pair(intersectionPoints.x, vertical.second.ContainerID));
                            }
                        }
                    }
                }
    
                vector<set<size_t>> orthogonal;
                bool passableVertical = lineIdentity(horizontalLines,verticalLines,orthogonal);
                    
                vector<set<size_t>> critical;
                bool passableHorizontal = lineIdentity(verticalLines,horizontalLines,critical);
                    
                if (passableVertical and passableHorizontal) {
                    vector<vector<Point2f>> corners(10,vector<Point2f>(10));
                    for(int i = 0; i< sudokuGridSize + 1; i++)
                        for(int j = 0; j < sudokuGridSize + 1; j++)
                            corners[i][j] = refinedIntersection(critical[i],orthogonal[j],horizontalLines,verticalLines);
                    
                            float reductionSize = 0.6;
                            vector<vector<pair<Point2f,Point2f>>> seperateBoxes(9,vector<pair<Point2f,Point2f>>(9));
                            for(int k = 0; k < sudokuGridSize; k++) {
                                for(int l = 0; l < sudokuGridSize; l++) {
                                    Point2f lowerBound = corners[k][l];
                                    Point2f upperBound = corners[k + 1][l + 1];
                                    
                                    
                                    float width = (upperBound.x - lowerBound.x) * reductionSize;
                                    float height = (upperBound.y - lowerBound.y) * reductionSize;
                                    float horizontalRefined = (upperBound.x + lowerBound.x)/2;
                                    float verticalRefined = (upperBound.y + lowerBound.y)/2;
                                    lowerBound.x = horizontalRefined - width /2;
                                    lowerBound.y = verticalRefined - height /2;
                                    upperBound.x = horizontalRefined + width /2;
                                    upperBound.y = verticalRefined + height /2;
                                    
                                    seperateBoxes[k][l].first = lowerBound;
                                    seperateBoxes[k][l].second = upperBound;
                                }
                    }
                    const int gridLength = 3;
                    Sudoku<gridLength> sudokuSolver;
                                    
                    // Sets hard-coded digits to the sudoku solver class, then renders to the video camera.
                    for(int i = 0; i < sudokuGridSize; i++)
                        for(int j = 0; j < sudokuGridSize; j++)
                            sudokuSolver.setValue(i, j, hardcodeDigits[i][j]);
                                    
                    if(sudokuSolver.solveSudoku()) {
                        for(int i = 0; i < sudokuGridSize; i++)
                            for(int j = 0; j < sudokuGridSize; j++) {
                                if (hardcodeDigits[i][j] == 0) {
                                    cv::Point textPosition(seperateBoxes[i][j].first.x +(seperateBoxes[i][j].second.x - seperateBoxes[i][j].first.x)/5,
                                    seperateBoxes[i][j].second.y-(seperateBoxes[i][j].second.y - seperateBoxes[i][j].first.y)/5);
                                    stringstream stringParse;
                                    stringParse << (int) sudokuSolver.getValue(i,j);
                                    putText(img, stringParse.str(), textPosition, CV_FONT_HERSHEY_DUPLEX, fontSize, Scalar(0,255,0), 1, 8);
                                }
                                NSLog(@"The number is: %d", hardcodeDigits[i][j]);
                            }
                        }
                }
}

/*
 * Data structure that will store intersection points, particularly around the sudoku board rectangles.
 */
struct RecognitionLines
{
    cv::Point criticalPoint1;
    cv::Point criticalPoint2;
    
    std::size_t ContainerID;
    
    multimap<double,size_t> intersections;
};

/*
 * Function that will find the intersection after given two points.
 */
bool intersection(Point2f offest1, Point2f point1, Point2f offset2, Point2f point2,
                  Point2f &intersectionPoint)
{
    Point2f x = offset2 - offest1;
    Point2f distanceOne = point1 - offest1;
    Point2f distanceTwo = point2 - offset2;
    
    float cross = distanceOne.x*distanceTwo.y - distanceOne.y*distanceTwo.x;
    if (abs(cross) < 1e-8)
        return false;
    
    double tDistance = (x.x * distanceTwo.y - x.y * distanceTwo.x)/cross;
    intersectionPoint = offest1 + distanceOne * tDistance;
    return true;
}

/*
 * Function identifies if line is horizontal or vertical.
 */
bool lineIdentity(const vector<pair<double,RecognitionLines>>& firstLine, const vector<pair<double,RecognitionLines>>& secondLine, vector<set<size_t>>& targetLines)
{
    double estimatedDistance = 20;
    
    if (firstLine.empty())
        return false;
    else
    {
        vector<pair<double,PairData>> numberDifference;
    
        size_t horiz_id = round(firstLine.size()/2);
        auto line_it = firstLine.begin()+horiz_id;
        const RecognitionLines& centeredLine = line_it->second;
        
        if (centeredLine.intersections.size() <= sudokuGridSize)
            return false;
        else
        {
            
            auto previousIntersection = centeredLine.intersections.begin();
            auto      currentIntersection = centeredLine.intersections.begin();
            currentIntersection++;
            for(; currentIntersection != centeredLine.intersections.end(); currentIntersection++, previousIntersection++)
                numberDifference.push_back(make_pair(currentIntersection->first-previousIntersection->first, PairData(previousIntersection->second, currentIntersection->second, currentIntersection->first)));
            
            sort(numberDifference.begin(),numberDifference.end(),[](const std::pair<double,PairData> &left, const std::pair<double,PairData> &right) {return left.first < right.first;});
            

            auto firstOccurance = numberDifference.begin();
            auto secondOccurance = numberDifference.begin()+8;
            double minDifference = 1000000;
            int minIndex = -1;
            size_t currentIndex = 0;
            for(;secondOccurance<numberDifference.end(); ++firstOccurance, ++secondOccurance, ++currentIndex)
            {
                if(firstOccurance->first>estimatedDistance)
                {
                    if(secondOccurance->first-firstOccurance->first<minDifference)
                    {
                        minDifference = secondOccurance->first-firstOccurance->first;
                        minIndex  = (int) currentIndex;
                    }
                }
            }
            

            if(minIndex < 0)
                return false;
            else if(max(numberDifference[minIndex].first,numberDifference[minIndex+8].first)/min(numberDifference[minIndex].first,numberDifference[minIndex+8].first) > 1.3)
                return false;
            else
            {
                vector<PairData> targetPairData(9);
                for(int i = 0; i < sudokuGridSize; i++)
                    targetPairData[i] = numberDifference[minIndex+i].second;
                sort(targetPairData.begin(),targetPairData.end(),[](const PairData &left, const PairData &right) {return left.intersection < right.intersection;});
                
                targetLines.resize(10);
                for(int i = 0; i < sudokuGridSize; i++)
                {
                    targetLines[i  ].insert(targetPairData[i].id1);
                    targetLines[i+1].insert(targetPairData[i].id2);
                }
            }
        }
    }
    
    return true;
}

/*
 * Function that will return intersection of lines after given horizontal and vertical lines.
 */
Point2f refinedIntersection(const set<size_t>& horizontalSet, const set<size_t>& verticalSet, const vector<pair<double,RecognitionLines>>& horizontal, const vector<pair<double,RecognitionLines>>& vertical)
{
    vector<Point2f> pointsAsInteger;
    for(auto iteration1:horizontalSet)
    {
        for(auto iteration2:verticalSet)
        {
            Point2f intersectionPoint;
            if(intersection(horizontal[iteration1].second.criticalPoint1, horizontal[iteration1].second.criticalPoint2,
                            vertical[iteration2].second.criticalPoint1, vertical[iteration2].second.criticalPoint2,
                            intersectionPoint))
                pointsAsInteger.push_back(intersectionPoint);
        }
    }
    
    Point2f avg = pointsAsInteger[0];
    for (int i = 1; i < pointsAsInteger.size(); i++)
        avg = avg + pointsAsInteger[i];
    avg.x = avg.x / (float) pointsAsInteger.size();
    avg.y = avg.y / (float) pointsAsInteger.size();
    return avg;
}


/*
 * Data structure that will hold positions of the sudoku squares.
 */
struct PairData
{
    PairData(size_t firstID = 0, size_t secondID = 0, double newIntersection = 0)
    : id1(firstID), id2(secondID), intersection(newIntersection)
    {
        
    }
    size_t id1;
    size_t id2;
    double intersection;
};

-(UIImage *) fixOrientation: (UIImage *) image {
    
    // No-op if the orientation is already correct
    if (image.imageOrientation == UIImageOrientationUp) return image;
    
    // We need to calculate the proper transformation to make the image upright.
    // We do it in 2 steps: Rotate if Left/Right/Down, and then flip if Mirrored.
    CGAffineTransform transform = CGAffineTransformIdentity;
    
    switch (image.imageOrientation) {
        case UIImageOrientationDown:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, image.size.height);
            transform = CGAffineTransformRotate(transform, M_PI);
            break;
            
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, 0);
            transform = CGAffineTransformRotate(transform, M_PI_2);
            break;
            
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, 0, image.size.height);
            transform = CGAffineTransformRotate(transform, -M_PI_2);
            break;
    }
    
    switch (image.imageOrientation) {
        case UIImageOrientationUpMirrored:
        case UIImageOrientationDownMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.width, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
            
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRightMirrored:
            transform = CGAffineTransformTranslate(transform, image.size.height, 0);
            transform = CGAffineTransformScale(transform, -1, 1);
            break;
    }
    
    // Now we draw the underlying CGImage into a new context, applying the transform
    // calculated above.
    CGContextRef ctx = CGBitmapContextCreate(NULL, image.size.width, image.size.height,
                                             CGImageGetBitsPerComponent(image.CGImage), 0,
                                             CGImageGetColorSpace(image.CGImage),
                                             CGImageGetBitmapInfo(image.CGImage));
    CGContextConcatCTM(ctx, transform);
    switch (image.imageOrientation) {
        case UIImageOrientationLeft:
        case UIImageOrientationLeftMirrored:
        case UIImageOrientationRight:
        case UIImageOrientationRightMirrored:
            // Grr...
            CGContextDrawImage(ctx, CGRectMake(0,0,image.size.height,image.size.width), image.CGImage);
            break;
            
        default:
            CGContextDrawImage(ctx, CGRectMake(0,0,image.size.width,image.size.height), image.CGImage);
            break;
    }
    
    // And now we just create a new UIImage from the drawing context
    CGImageRef cgimg = CGBitmapContextCreateImage(ctx);
    UIImage *img = [UIImage imageWithCGImage:cgimg];
    CGContextRelease(ctx);
    CGImageRelease(cgimg);
    return img;
}

/*
 * Properly converts from UIImage to cv:Mat frame.
 */
cv::Mat cvMatFromUIImage(UIImage *image)
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}

//Ref:Open CV documentation
/*
 * Properly converts from cv:Mat frame to UIImage.
 */
UIImage* imageWithCVMat(cv::Mat cvMat)
{
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize() * cvMat.total()];
    
    CGColorSpaceRef colorSpace;
    
    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                     // Width
                                        cvMat.rows,                                     // Height
                                        8,                                              // Bits per component
                                        8 * cvMat.elemSize(),                           // Bits per pixel
                                        cvMat.step[0],                                  // Bytes per row
                                        colorSpace,                                     // Colorspace
                                        kCGImageAlphaNone | kCGBitmapByteOrderDefault,  // Bitmap info flags
                                        provider,                                       // CGDataProviderRef
                                        NULL,                                           // Decode
                                        false,                                          // Should interpolate
                                        kCGRenderingIntentDefault);                     // Intent
    
    UIImage *image = [[UIImage alloc] initWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return image;
}

- (void)start
{
    [videoCamera start];
}

- (void)stop
{
    [videoCamera stop];
}
@end
