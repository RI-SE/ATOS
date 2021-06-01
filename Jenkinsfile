pipeline {
  agent any 
  stages {
    stage('Build') {
            steps {

          checkout(
              [
                $class: 'GitSCM', 
                branches: [
                  [
                    name: '*/master'
                  ]
                ], 
                doGenerateSubmoduleConfigurations: false, 
                extensions: [
                  [
                    $class: 'SubmoduleOption', 
                    disableSubmodules: false, 
                    parentCredentials: true, 
                    recursiveSubmodules: true, 
                    reference: '', 
                    trackingSubmodules: false
                  ]
                ], 
                submoduleCfg: [], 
                userRemoteConfigs: [
                  [
                    credentialsId: 'JenkinsUser', 
                    url: 'git@github.com:RI-SE/Maestro.git'
                  ]
                ]
              ]
            )
            }
    }
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

