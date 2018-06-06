//
//  PhotoViewController.swift
//  sudokuSolver
//
//  Created by Gary Liang on 5/5/18.
//  Copyright © 2018 Gary Liang. All rights reserved.
//

import UIKit
import Vision


class PhotoViewController: UIViewController {

    var takenPhoto:UIImage?
    
    //@IBOutlet weak var vision: UILabel!
    @IBOutlet weak var imageView: UIImageView!
    //@IBOutlet weak var classificationLabel: UILabel!
    
    override func viewDidLoad() {
        //super.viewDidLoad()
        // Do any additional setup after loading the view.
        //setupLabel()
        if let availableImage = takenPhoto {
            imageView.image = availableImage
        }
    }
    
    /*
     Action event for back button.
     */
    @IBAction func goBack(_ sender: Any) {
        
        self.dismiss(animated:true, completion: nil)
    }
    
    //func setupLabel() {
    //    classificationLabel.translatesAutoresizingMaskIntoConstraints = false
    //    classificationLabel.font = classificationLabel.font.withSize(30)
    //    classificationLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor).isActive = true
    //    classificationLabel.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -50).isActive = true
    //}
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destinationViewController.
        // Pass the selected object to the new view controller.
    }
    */

}
