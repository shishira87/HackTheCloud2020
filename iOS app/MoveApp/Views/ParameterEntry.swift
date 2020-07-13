//
//  ParameterEntry.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/12/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct ParameterEntry : View {
    
    @State var ccode = ""
    @State var x = ""
    @State var y = ""
    @State var z = ""
    @State var show = false
    @State var msg = ""
    @State var alert = false
    @State var ID = ""
    @Environment(\.viewController) private var viewControllerHolder: UIViewController?
    
    var body : some View{
        
        VStack(spacing: 20){
            
            Image("robot").resizable().frame(width: 200, height: 200)
            
            Text("Create New Path").font(.largeTitle).fontWeight(.heavy)
            
            Text("Enter the parameters for your garage")
                .font(.body)
                .foregroundColor(.gray)
                .padding(.top, 12)
            
            HStack{
                
                TextField("X", text: $x)
                .frame(width: UIScreen.main.bounds.width - 300,height: 20)
                    .keyboardType(.numberPad)
                    .padding()
                    .background(Color("input"))
                    .clipShape(RoundedRectangle(cornerRadius: 10))
                   
                
                TextField("Y", text: $y)
                .frame(width: UIScreen.main.bounds.width - 300,height: 20)
                    .keyboardType(.numberPad)
                    .padding()
                    .background(Color("input"))
                    .clipShape(RoundedRectangle(cornerRadius: 10))
                
            
                TextField("Z", text: $z)
                .frame(width: UIScreen.main.bounds.width - 300,height: 20)
                    .keyboardType(.numberPad)
                    .padding()
                    .background(Color("input"))
                    .clipShape(RoundedRectangle(cornerRadius: 10))
                
            } .padding(.top, 15)

//            NavigationLink(destination: ScndPage(show: $show, ID: $ID), isActive: $show) {
                
                
                Button(action: {
                    self.viewControllerHolder?.present(style: .fullScreen) {
                        ConfirmationScreen()
                    }
                }) {
                    
                    Text("Create Path").frame(width: UIScreen.main.bounds.width - 30,height: 50)
                    
                }.foregroundColor(.white)
                .background(Color.orange)
                .cornerRadius(10)
//            }

            .navigationBarTitle("")
            .navigationBarHidden(true)
            .navigationBarBackButtonHidden(true)
            
        }.offset(y: -80)
        .alert(isPresented: $alert) {
                
            Alert(title: Text("Error"), message: Text(self.msg), dismissButton: .default(Text("Ok")))
        }
    }
}

struct ParameterEntry_Previews: PreviewProvider {
    static var previews: some View {
        ParameterEntry()
    }
}
