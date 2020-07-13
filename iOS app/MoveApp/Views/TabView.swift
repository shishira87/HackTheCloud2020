//
//  TabView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct TabViewIndex: View {
//    @State var index = 0
    @State private var selection = 0
        var body: some View {
            
            TabView {
            
                Home().tabItem {
                        Image("homeFill").font(.title)
                }
                
                ParameterEntry().tabItem {
                    
                    Image("road").font(.title)
                }
                
                Text("user").tabItem {
                    
                    Image("user").font(.title)
                }
            }
        }
}

struct TabView_Previews: PreviewProvider {
    static var previews: some View {
        TabViewIndex()
    }
}
