//
//  OnOffDetail.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/13/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct OnOffDetail : View {
    
        @Environment(\.viewController) private var viewControllerHolder: UIViewController?
    
    var body : some View{
        
        VStack(spacing: 50){
            
            Image("control").resizable().frame(width: 273, height: 186)
            
            Text("Control Center").font(.largeTitle).fontWeight(.heavy)
            
            Text("Pause execution by pressing 'start' or 'stop'")
                .font(.body)
                .foregroundColor(.gray)
                .padding(.top, 12)
            


                
                
                Button(action: {
                }) {
                    
                    Text("Start").frame(width: UIScreen.main.bounds.width - 80,height: 50)
                    
                }.foregroundColor(.black)
                .background(Color("green"))
                .cornerRadius(10)
                    
            Button(action: {
                self.viewControllerHolder?.present(style: .fullScreen) {
                    TabViewIndex()
                }
            }) {
                
                Text("Stop").frame(width: UIScreen.main.bounds.width - 80,height: 50)
                
            }.foregroundColor(.black)
            .background(Color("red"))
            .cornerRadius(10)

            .navigationBarTitle("")
            .navigationBarHidden(true)
            .navigationBarBackButtonHidden(true)
            
        }.offset(y: -80)
    }
}

