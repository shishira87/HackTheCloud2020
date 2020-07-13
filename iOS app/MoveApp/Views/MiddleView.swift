//
//  MiddleView.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
import SwiftUI


struct Middle : View {
    @State var show = false
    var body : some View {
        
        ScrollView(.horizontal, showsIndicators: false) {
            
            HStack(spacing: 20) {
                VStack(alignment: .leading, spacing: 12) {
                    Button(action: {
                        
                    }) {
                        ZStack {
                            Rectangle()
                                .fill(Color("purple"))
                                .frame(width: 180, height: 240)
                            .cornerRadius(10)

                            Image("food").renderingMode(.original)
                        }
                        
                    }
                    
                    Text("Mom's House").fontWeight(.heavy)
                    
                    HStack(spacing: 5) {
                        Text("Garage Door 1").foregroundColor(.gray)
                    }
                }
                
                VStack(alignment: .leading, spacing: 12) {
                    Button(action: {
                        
                    }) {
                            ZStack {
                                Rectangle()
                                    .fill(Color("bg"))
                                    .frame(width: 180, height: 240)
                                .cornerRadius(10)

                                Image("food").renderingMode(.original)
                            }
                            
                        }
                    
                    Text("Cousin's House").fontWeight(.heavy)
                    
                    HStack(spacing: 5) {
                        Text("Garage Door 3").foregroundColor(.gray)
                    }
                }
                
                VStack(alignment: .leading, spacing: 12) {
                    Button(action: {
                        self.show.toggle()
                    }) {
                        ZStack {
                                Rectangle()
                                    .fill(Color("teal"))
                                    .frame(width: 180, height: 240)
                                .cornerRadius(10)

                                Image("food").renderingMode(.original)
                            }
                            
                        
                    }
                    
                    Text("123 Main Street House").fontWeight(.heavy)
                    
                    HStack(spacing: 5) {
                        Text("Garage Door 1").foregroundColor(.gray)
                    }
                }
            }
        }.sheet(isPresented: $show) {
            OnOffDetail()
        }
    }
}







struct Rounded : Shape {
    
    func path(in rect: CGRect) -> Path {
        
        let path = UIBezierPath(roundedRect: rect, byRoundingCorners: [.topLeft,.topRight], cornerRadii: CGSize(width: 40, height: 40))
        return Path(path.cgPath)
    }
}


struct MiddleView_Previews: PreviewProvider {
    static var previews: some View {
       OnOffDetail()
    }
}
