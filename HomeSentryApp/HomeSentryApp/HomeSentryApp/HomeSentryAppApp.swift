//
//  HomeSentryAppApp.swift
//  HomeSentryApp
//
//  Created by Maxime BOULANGER on 17/12/2023.
//

import SwiftUI

@main
struct HomeSentryAppApp: App {
    let persistenceController = PersistenceController.shared

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(\.managedObjectContext, persistenceController.container.viewContext)
        }
    }
}
