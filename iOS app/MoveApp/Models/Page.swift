//
//  Page.swift
//  MoveApp
//
//  Created by Rohan  Minocha on 7/11/20.
//  Copyright © 2020 Rohan Minocha. All rights reserved.
//

import Foundation
struct Page: Identifiable {
    
    let id: UUID
    let image: String
    let heading: String
    let subSubheading: String
    
    static var getAll: [Page] {
        [
            Page(id: UUID(), image: "screen-1", heading: "Simplicity at Home", subSubheading: "Allow MOVE to help manage the difficulties you face when moving heavy goods. "),
            Page(id: UUID(), image: "screen-2", heading: "Easy to set up", subSubheading: "MOVE is designed to be autonomous in order to be accomodating to anyone, whether they suffer from injuries, disabilities, or fragility brought upon by old age"),
            Page(id: UUID(), image: "screen-3", heading: "Versatile applications", subSubheading: "MOVE is designed with versatility in mind, so that it is extensible and applicable to all object carrying needs"),
            Page(id: UUID(), image: "screen-4", heading: "No More Struggles", subSubheading: "MOVE removes the struggles from your daily life so that you can focus on your own health and safety")
            
        ]
    }
}
