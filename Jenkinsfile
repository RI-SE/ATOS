
pipeline {
    agent { docker { image 'python:3.5.1' } }
    environment {
        PATH = "/usr/bin"
    }
    stages {
        stage('build') {
            steps {
                sh 'echo "Executing build script..."'
                sh './buildMaestro.sh'
            }
        }
    }
}


