//
//  BottomView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/13/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct BottomView : View {
    
    var body : some View{
        
        VStack{
            
            HStack{
                
                Text("Previous Paths").fontWeight(.heavy)
                
                Spacer()
                
                Button(action: {
                    
                }) {
                    
                    Text("View all").foregroundColor(.gray)
                }
                
            }.padding([.top], 15)
            
            ScrollView(.horizontal, showsIndicators: false) {
                
                HStack(spacing: 35){
                    
                    Button(action: {
                            
                    }) {
                            
                        VStack(spacing: 8){
                                
                            ZStack {
                                Rectangle()
                                    .fill(Color("pastelYellow"))
                                    .frame(width: 60, height: 60)
                                .cornerRadius(10)

                                Image("battery")
                                    .resizable().frame(width: 40, height: 40)
                                    .accentColor(.black)
                                    .rotationEffect(Angle(degrees: 270))

                            }
                            
                            Text("").foregroundColor(.gray)
                        }
                    }
                    
                    Button(action: {
                            
                    }) {
                            
                    VStack(spacing: 8){
                                
                            ZStack {
                                Rectangle()
                                    .fill(Color("pastelGreen"))
                                    .frame(width: 60, height: 60)
                                .cornerRadius(10)

                                Text("2")
                                    .foregroundColor(.black)
                            }
                            
                            Text("").foregroundColor(.gray)
                        }
                    }
                        
                    Button(action: {
                            
                    }) {
                            
                        VStack(spacing: 8){
                                
                            ZStack {
                                Rectangle()
                                    .fill(Color("salmon"))
                                    .frame(width: 60, height: 60)
                                .cornerRadius(10)

                                Text("3")
                                    .foregroundColor(.black)
                            }
                            
                            Text("").foregroundColor(.gray)
                        }
                    }
                    Button(action: {
                            
                    }) {
                            
                        VStack(spacing: 8){
                                
                            ZStack {
                                Rectangle()
                                    .fill(Color("pastelBlue"))
                                    .frame(width: 60, height: 60)
                                .cornerRadius(10)

                                Text("4")
                                    .foregroundColor(.black)
                            }
                            
                            Text("").foregroundColor(.gray)
                        }
                    }
                }
            }.padding(.leading, 20)
            .padding([.top,.bottom], 15)
        }
    }
}
