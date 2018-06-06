//
//  OpenCVWrapper.h
//  sudokuSolver
//
//  Created by Gary Liang on 5/12/18.
//  Copyright © 2018 Gary Liang. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <TesseractOCR/TesseractOCR.h>
#import "leptonica/src/allheaders.h"


// Protocol for callback action
@protocol SudokuCameraDelegate <NSObject>


@end

@interface SudokuCamera : NSObject
    -(id) initWithController: (UIViewController<SudokuCameraDelegate>*)c andImageView: (UIImageView*)iv;
    -(UIImage *) fixOrientation: (UIImage *) image;
    -(void)start;
    -(void)stop;
    
@end
