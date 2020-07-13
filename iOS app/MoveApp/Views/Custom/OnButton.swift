//
//  OnButton.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/12/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct OnButton: View {
    var text = "Next"
    
    var body: some View {
      Button(action: {
        
      }) {
        HStack {
            Text(text)
                .bold()
                .frame(minWidth: 0, maxWidth: .infinity)
                .padding(.vertical)
                .accentColor(Color.white)
                .background(Color("red"))
                .cornerRadius(10)
            }
        }
    }
}

struct OnButton_Previews: PreviewProvider {
    static var previews: some View {
        OnButton()
    }
}
