//
//  LoginView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI





struct LoginView: View {
        
    @State private var name = ""
    @State private var formOffset: CGFloat = 0
    @State private var presentSignupSheet = false
    
        var body: some View {
            VStack(spacing: 40) {
                Image("login")
                Text("Sign Up").font(.title).bold()
                VStack {
                    NameEntry(value: self.$name, placeholder: "Name",  onEditingChanged: { flag in
                        withAnimation {
                            self.formOffset = flag ? -150 : 0
                        }
                    })

                    ProceedButton()
                }
                
            
                
            }.padding()
                .offset(y: self.formOffset)
        }
    }

