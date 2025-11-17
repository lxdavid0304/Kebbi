pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()

    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()

        // Kotlin DSL ─ flatDir 要這樣寫 ↓
        flatDir {
            dirs("C:\\MyApplication12\\app\\src\\main\\libs")
        }
    }
}

rootProject.name = "My Application12"
include(":app")
 