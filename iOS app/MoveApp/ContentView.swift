//
//  ContentView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import SwiftUI

struct ContentView: View {
    @State var show = false
    @State var index = 0

    // set to false in prod
    private let initialLaunchKey = "isInitialLaunch"
    
    var body: some View {
        VStack {
            // change false to UserDefaults.standard.bool(forKey: initialLaunchKey)
            if show {
                LoginView().transition(.move(edge: .bottom))
            } else {
                PageViewContainer( viewControllers: Page.getAll.map({UIHostingController(rootView: PageView(page: $0) )}), presentSignupView: {
                    withAnimation {
                        self.show = true
                    }
                    UserDefaults.standard.set(true, forKey: self.initialLaunchKey)
                }).transition(.scale)
            }
        }.frame(maxHeight: .infinity)
            .background(Color.backgroundColor)
            .edgesIgnoringSafeArea(.all)
            .onTapGesture {
                UIApplication.shared.endEditing()
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
