//
//  ProceedButton.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI

struct ProceedButton: View {
    @Environment(\.viewController) private var viewControllerHolder: UIViewController?
    
    var body: some View {
      Button(action: {
        self.viewControllerHolder?.present(style: .fullScreen) {
            TabViewIndex()
        }
      }) {
        HStack {
            Text("Let's Go")
                .bold()
                .frame(minWidth: 0, maxWidth: .infinity)
                .padding(.vertical)
                .accentColor(Color.white)
                .background(Color("red"))
                .cornerRadius(15)
            }
        }
    }
}

struct LCButton_Previews: PreviewProvider {
    static var previews: some View {
        ProceedButton()
    }
}



struct ViewControllerHolder {
    weak var value: UIViewController?
}

struct ViewControllerKey: EnvironmentKey {
    static var defaultValue: ViewControllerHolder {
        return ViewControllerHolder(value: UIApplication.shared.windows.first?.rootViewController)

    }
}


