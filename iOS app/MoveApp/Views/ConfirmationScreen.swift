//
//  ConfirmationScreen.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/12/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct ConfirmationScreen : View {
    
    @State var ccode = ""
    @State var no = ""
    @State var show = false
    @State var msg = ""
    @State var alert = false
    @State var ID = ""
    @Environment(\.viewController) private var viewControllerHolder: UIViewController?
    
    var body : some View{
        
        VStack(spacing: 20){
            
            Image("robot").resizable().frame(width: 200, height: 200)
            
            Text("Thank you").font(.largeTitle).fontWeight(.heavy)
            
            Text("Your path has been created")
                .font(.body)
                .foregroundColor(.gray)
                .padding(.top, 12)
            

            .navigationBarTitle("")
            .navigationBarHidden(true)
            .navigationBarBackButtonHidden(true)
            
        }.padding()
        .alert(isPresented: $alert) {
                
            Alert(title: Text("Error"), message: Text(self.msg), dismissButton: .default(Text("Ok")))
        }
    }
}

struct ConfirmationScreen_Previews: PreviewProvider {
    static var previews: some View {
        ConfirmationScreen()
    }
}
