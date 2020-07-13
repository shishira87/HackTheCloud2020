//
//  Home.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct Home : View {
    
    var body : some View{
        
        VStack(alignment: .leading,spacing: 12){
            
            HStack{
                
                Button(action: {
                    
                }) {
                    
                    Text("")
                }
                
                Spacer()
                
                Button(action: {
                    
                }) {
                    
                    Text("")
                }
            }
            
            Text("Hello, Rohan").fontWeight(.heavy).font(.largeTitle).padding(.top,15)

        
            Middle()
            
            BottomView().padding(.top, 10)
            
        }.padding()
    }
}

struct Home_Previews: PreviewProvider {
    static var previews: some View {
       Home()
    }
}
