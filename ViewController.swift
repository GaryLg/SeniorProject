//
//  ViewController.swift
//  sudokuSolver
//
//  Created by Gary Liang on 5/5/18.
//  Copyright © 2018 Gary Liang. All rights reserved.
//

import UIKit
import AVFoundation
import Vision
import CoreML
import ImageIO
import TesseractOCR

class ViewController: UIViewController, SudokuCameraDelegate {

    let captureSession = AVCaptureSession()
    var previewLayer:CALayer!
    var captureDevice:AVCaptureDevice!
    var inputImage: CIImage! // The image to be processed.
    var camera: SudokuCamera!
    @IBOutlet var imgView: UIImageView!
    var takePhoto = false
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        camera = SudokuCamera(controller: self, andImageView: imgView)
        
        
        
        
    }
    
    
    override func viewDidAppear(_ animated: Bool) {
        camera.start()
    }
    
    // Stop it when it disappears
    override func viewWillDisappear(_ animated: Bool) {
        camera.stop()
    }

    
    /*
     Prepares the camera. Preset and other defined settings for camera is defined here.
     */
    func prepareCamera() {
        captureSession.sessionPreset = AVCaptureSession.Preset.photo
        
        if let availableDevices = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera], mediaType: AVMediaType.video, position: .back).devices as? [AVCaptureDevice] {
            captureDevice = availableDevices.first
            beginSession()
        }
    }
    
    func beginSession() {
        do {
            let captureDeviceInput = try AVCaptureDeviceInput(device: captureDevice)
            captureSession.addInput(captureDeviceInput)
        } catch {
            print("Cannot detect any camera.")
        }
        
        //may need fixing
        
        if let newPreviewLayer = AVCaptureVideoPreviewLayer(session: captureSession) as? CALayer {
            self.previewLayer = newPreviewLayer
            self.view.layer.addSublayer(self.previewLayer)
            self.previewLayer.frame = self.view.layer.frame
            captureSession.startRunning()
            
            //outputting photo
            let dataOutput = AVCaptureVideoDataOutput()
            dataOutput.videoSettings = [(kCVPixelBufferPixelFormatTypeKey as NSString):NSNumber(value:kCVPixelFormatType_32BGRA)] as [String : Any]
            
            dataOutput.alwaysDiscardsLateVideoFrames = true
            
            
            if captureSession.canAddOutput(dataOutput) {
                captureSession.addOutput(dataOutput)
            }
            
            captureSession.commitConfiguration()
            
           
        }
            
        
    }
    
    @IBAction func takePhoto(_ sender: Any) {
        takePhoto = true
    }
    
    /*
        Outputs taken photo to buffer. Method will also use Vision API for image recognition.
     */
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        
        if takePhoto {
            //takePhoto = false
            if let image = self.getImageFromSampleBuffer(buffer: sampleBuffer) {

            }
        }
    }

    
    func getImageFromSampleBuffer (buffer:CMSampleBuffer) -> UIImage? {
        if let pixelBuffer = CMSampleBufferGetImageBuffer(buffer) {
            let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
            let context = CIContext()
            let imageRect = CGRect(x: 0, y: 0, width: CVPixelBufferGetWidth(pixelBuffer), height: CVPixelBufferGetHeight(pixelBuffer))
            
            if let image = context.createCGImage(ciImage, from: imageRect){
                return UIImage(cgImage: image,  scale:UIScreen.main.scale, orientation: .right)
                
            }
        }
        
        return nil
    }
    
    
    func stopCaptureSession() {
        self.captureSession.stopRunning()
        if let inputs = captureSession.inputs as? [AVCaptureDeviceInput] {
            for input in inputs {
                self.captureSession.removeInput(input)
            }
        }
    }
    

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }


}

