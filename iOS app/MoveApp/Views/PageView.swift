//
//  PageView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import SwiftUI
import SwiftUI

struct PageView: View {
    
    var page = Page.getAll.first!
    
    var body: some View {
        VStack(spacing: 50){
                
                Image(page.image)
                .resizable()
                .scaledToFill() // add if you need
                .frame(width: 300, height: 150) 
                VStack{
                    Text(page.heading).font(.title).bold().layoutPriority(1).multilineTextAlignment(.center)
                    Text(page.subSubheading)
                        .multilineTextAlignment(.center)
                }.padding()
            }
        
    }
}

struct PageView_Previews: PreviewProvider {
    static var previews: some View {
        PageView()
    }
}
