//
//  TabBar.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI


struct TabBar : View {
    @Binding var index : Int
    var body : some View {
        HStack {
            HStack {
                Image(systemName: "house.fill").resizable().frame(width: 35, height: 30)
                Text(self.index == 0 ? "Home" : "")
                    .fontWeight(.light).font(.system(size: 14))
            }.padding(9.0)
                .background(self.index == 0 ? Color.red.opacity(0.5) : Color.clear)
            .clipShape(Capsule())
                .onTapGesture {
                    self.index = 0
            }
            
            HStack {
                Image(systemName: "person.fill").resizable().frame(width: 20, height: 20)
                
                Text(self.index == 1 ? "Stats" :
                    "").fontWeight(.light).font(.system(size: 14))
            }.padding(15)
                .background(self.index == 1 ? Color.blue.opacity(0.5) : Color.clear)
            .clipShape(Capsule())
                .onTapGesture {
                    self.index = 1
            }
            
            HStack {
                Image(systemName: "bell.fill").resizable().frame(width: 30, height: 30)
                
                Text(self.index == 2 ? "Settings" :
                    "").fontWeight(.light).font(.system(size: 14))
            }.padding(15)
                .background(self.index == 2 ? Color.green.opacity(0.5) : Color.clear)
            .clipShape(Capsule())
                .onTapGesture {
                    self.index = 2
            }
            
            HStack {
                Image(systemName: "tv.fill").resizable().frame(width: 30, height: 30)

                Text(self.index == 3 ? "Watch" :
                    "").fontWeight(.light).font(.system(size: 14))
            }.padding(15)
                .background(self.index == 3 ? Color.yellow.opacity(0.5) : Color.clear)
            .clipShape(Capsule())
                .onTapGesture {
                    self.index = 3
            }
        
        }.padding(.top, -10)
    .frame(width: UIScreen.main.bounds.width)
            .background(Color.white)
            .animation(.default)
  }
}
