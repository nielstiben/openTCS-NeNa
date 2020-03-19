package de.fraunhofer.iml.opentcs.scripts

import java.util.regex.Pattern
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.*
import java.io.FileFilter
import org.apache.commons.io.FileUtils
import groovy.io.FileType

// Path relative to the root project's build.gradle
String basePath = ''
String oldIntegrationName = 'Example'
String newIntegrationName = args[0]
String oldClassPrefix = 'Example'
String newClassPrefix = args[1]

// The path to the source project
Path sourcePath = new File(basePath).toPath().toAbsolutePath()
// Create a directory with the new project name
File destFile = new File('build/openTCS-Integration-' + newIntegrationName)
// The path to the (new) destination project
Path destPath = destFile.toPath().toAbsolutePath()

// Copy the project files but apply a file filter
FileUtils.copyDirectory(sourcePath.toFile(), destPath.toFile(), new ExcludeFilter(sourcePath))

// Rename the copied directories
destFile.eachFile { file ->
  if (file.isDirectory() && file.name.contains('-' + oldIntegrationName + '-')) {
    String newName = file.getPath().replace('-' + oldIntegrationName + '-', '-' + newIntegrationName + '-')
    file.renameTo(newName)
  }
}

// Change file content for the root project's build.gradle
File buildGradle = new File(destPath.resolve('build.gradle').toString())
buildGradle.text = buildGradle.text.replace(oldIntegrationName, newIntegrationName)
                                   .replaceAll(Pattern.compile("\\Rdependencies\\s\\{([^}]|\\R)+\\}\\R"), '') // Remove the 'dependencies' block. We don't need the groovy dependency.
                                   .replace('apply plugin: \'groovy\'', '') // We also don't need the groovy plugin
                                   .replaceAll(Pattern.compile("\\R(\\/\\/ tag::cloneTask)((.|\\R)*)\\1\\R"), ''); // Remove the 'cloneProject' task

// Change file content for the root project's settings.gradle
File settingsGradle = new File(destPath.resolve('settings.gradle').toString())
settingsGradle.text = settingsGradle.text.replace(oldIntegrationName, newIntegrationName)

// Change file content for all build.gradle files
destFile.eachFileRecurse(FileType.FILES){ file ->
  if (file.name.equals('build.gradle')) {
    file.text = file.text.replace(oldIntegrationName, newIntegrationName)
  }
}

destFile.eachFileRecurse(FileType.FILES){ file ->
  // Change file name and content for all java classes
  if (file.name.matches("(.+).java")) {
    // Change the content
    file.text = file.text.replace(oldClassPrefix, newClassPrefix)
    
    // Change the file name
    if (file.name.matches("^" + oldClassPrefix + "(.+)")) {
      String newName = file.getPath().replace(oldClassPrefix, newClassPrefix)
      file.renameTo(newName)
    }
  }
  
  // Change content of some Guice-specific files
  if (file.name.matches("(.+).KernelInjectionModule")
      || file.name.matches("(.+).ControlCenterInjectionModule")) {
    file.text = file.text.replace(oldClassPrefix, newClassPrefix)
  }
  
  // Change content of resource bundles
  if (file.name.matches("Bundle(.*).properties")) {
    file.text = file.text.replace(oldClassPrefix, newClassPrefix)
  }
}


class ExcludeFilter implements FileFilter {
  
  private Path excludeFrom;
  private List<String> toExclude = [".git", ".gitignore", ".gradle", ".nb-gradle", "build", "src", "README.md"];
  
  ExcludeFilter(Path excludeFrom) {
    this.excludeFrom = excludeFrom;
  }
  
  @Override
  public boolean accept(File file) {
    for (String exclude : toExclude) {
      Path excludePath = excludeFrom.resolve(exclude)
      if (Files.exists(excludePath) && Files.isSameFile(excludePath, file.toPath())) {
        return false;
      }
    }
    return true;
  }
}