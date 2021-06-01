pipeline {
  agent any
  stages {
    stage('Checkout'){
      steps{
          checkout([$class: 'GitSCM', branches: [[name: '*/master']], extensions: [[$class: 'CheckoutOption', timeout: 50], [$class: 'SubmoduleOption', disableSubmodules: false, parentCredentials: true, recursiveSubmodules: true, reference: '', timeout: 50, trackingSubmodules: false]], userRemoteConfigs: [[credentialsId: '4c683913-52c5-4e02-a46f-36e5b9a14903', url: 'https://github.com/RI-SE/Maestro/']]])
      }
    }
    
    stage('Build') {
      steps {
        sh 'echo "Executing build steps..."'
        cmakeBuild(cleanBuild: true, buildDir: 'build', installation: 'InSearchPath', steps: [[envVars: 'DESTDIR=${WORKSPACE}/artifacts', withCmake: true]])
      }
    }

    stage('Run tests') {
      parallel {
        stage('Master Integration tests') {
          when {
            expression {
              GIT_BRANCH = 'origin/' + sh(returnStdout: true, script: 'git rev-parse --abbrev-ref HEAD').trim()
              return GIT_BRANCH == 'origin/master'
            }

          }
          steps {
            sh 'echo "Running extensive Maestro integration tests..."'
            sh './maestroExtensiveTests.sh'
          }
        }

        stage('Dev Integration tests') {
          steps {
            sh 'echo "Running standard Maestro integration tests..."'
            sh './maestroStandardTests.sh'
          }
        }

        stage('Format check') {
          steps {
            sh 'echo "Running code formatting check..."'
            sh './checkCodeFormat.sh'
          }
        }

      }
    }

  }
  options {
    timeout(time: 15, unit: 'MINUTES')
  }
}
