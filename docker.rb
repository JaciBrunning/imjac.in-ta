require 'fileutils'

DOCKER_TAG="#{`git rev-parse --short HEAD`.strip}#{`git diff HEAD --quiet || echo -dirty`.strip}"
DOCKER_IMG="gcr.io/imjacinta/jaci/imjacinta:#{DOCKER_TAG}"

def copy_deps
  FileUtils.mkdir_p 'build/depslayer'
  Dir.glob(['**/Gemfile', '**/Gemfile.lock', '**/*.gemspec']).each do |file|
    next if file.include?('build/depslayer')
    
    dest = "build/depslayer/#{file}"
    FileUtils.mkdir_p(File.dirname(dest))
    
    if !File.exist?(dest) || File.read(file) != File.read(dest)
      puts "Update: #{file} -> #{dest}"
      FileUtils.cp(file, dest) 
    end
  end
end

def exec_machine prefix, command
  name = ENV['deploy_name'] || 'imjacinta'
  status = `docker-machine status #{name}`
  raise "Machine does not exist! Use rake docker:create_deployment to make one!" if status.empty?

  command = "#{prefix} ./run_in_machine.sh #{name} #{command}"
  exec command
end

def create_deployment
  key = ENV['key'] || '~/.ssh/id_rsa'
  name = ENV['deploy_name'] || 'imjacinta'
  user = ENV['sshuser'] || 'root'
  host = ENV['ip']

  raise "No IP given! Provide it with 'ip=XX.XX.XX.XX' as an env var" if host.nil? || host.empty?

  puts "Creating Docker-Machine with IP: #{host}, Name: #{name}, SSH Key: #{key}, SSH User: #{user}"
  exec "docker-machine create --driver generic --generic-ip-address #{host} --generic-ssh-user #{user} --generic-ssh-key #{key} #{name}"
end

def docker_build
  copy_deps
  system "docker build -t #{DOCKER_IMG} ."
end

def docker_push
  docker_build
  system "docker push #{DOCKER_IMG}"
end

def docker_up_dev
  copy_deps
  system "docker-compose up --build"
end

def docker_deploy
  exec_machine "IMJACINTA_VERSION=#{DOCKER_TAG}", "docker stack deploy --compose-file=docker-compose-prod.yml imjacinta"
end

if __FILE__ == $0
  act = ARGV[0]
  if act == 'build'
    docker_build
  elsif act == 'push'
    docker_push
  elsif act == 'up'
    docker_up_dev
  elsif act == 'deploy'
    docker_deploy
  elsif act == 'create_deploy'
    create_deployment
  elsif act == 'get_tag'
    puts DOCKER_TAG
  elsif act == 'get_img'
    puts DOCKER_IMG
  end
end