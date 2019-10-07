
pipeline {
    agent any
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


