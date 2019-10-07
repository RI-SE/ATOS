
pipeline {
    agent any
   
    stages {
        stage('Build') {
            steps {
                sh 'echo "Executing build script..."'
                sh './buildMaestro.sh'
            }
        }
        stage('Integration testing') {
            steps {
                sh 'echo "Running Maestro integration tests..."'
                sh './allMaestroIntegrationTests.sh'
            }
        }
    }
}


