//
//  LCTextField.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct NameEntry: View {
    
    @Binding var value: String
    var placeholder = "Placeholder"
    var color = Color("offColor")
    var isSecure = false
    var onEditingChanged: ((Bool)->()) = {_ in }
    
    var body: some View {
        HStack {
            
            if isSecure{
                SecureField(placeholder, text: self.$value, onCommit: {
                    self.onEditingChanged(false)
                }).padding()
            } else {
                TextField(placeholder, text: self.$value, onEditingChanged: { flag in
                    self.onEditingChanged(flag)
                }).padding()
            }

        }.background(color.opacity(0.2)).clipShape(Capsule())
    }
}

struct LCTextfield_Previews: PreviewProvider {
    static var previews: some View {
        NameEntry(value: .constant(""))
    }
}
