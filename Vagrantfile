# -*- mode: ruby -*-
# vi: set ft=ruby :

require 'socket'

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "precise64"

  config.vm.hostname = "pdal-vagrant"
  config.vm.box_url = "http://files.vagrantup.com/precise64.box"
  config.vm.host_name = "pdal-vagrant"
  
  config.vm.network :forwarded_port, guest: 80, host: 8080

  config.vm.provider :virtualbox do |vb|
     vb.customize ["modifyvm", :id, "--memory", "1024"]
     vb.customize ["modifyvm", :id, "--cpus", "2"]   
     vb.customize ["modifyvm", :id, "--ioapic", "on"]
     vb.name = "pdal-vagrant"
   end  


  if RUBY_PLATFORM.include? "darwin"
    config.vm.network "private_network", ip: "192.168.50.4"
    config.vm.synced_folder ".", "/vagrant", nfs: true
    
    if Socket.gethostname.include? "pyro" # Howard's machine
     config.vm.synced_folder "/Users/hobu/dev/git/pointcloud", "/pointcloud", nfs: true
    end
  end
  
  

  ppaRepos = [
	  "ppa:ubuntugis/ppa",
    "ppa:apokluda/boost1.53"
  ]

  packageList = [
	  "git",
	  "build-essential",
	  "pkg-config",
	  "cmake",
    "libgeos-dev",
	  "libgdal-dev",
	  "libpq-dev",
    "python-all-dev",
    "python-numpy",
	  "libproj-dev",
	  "libtiff4-dev",
	  "libxml2-dev",
    "libboost1.53-all-dev",
    "libbz2-dev",
    "libsqlite0-dev",
    "cmake-curses-gui",
    "screen",
    "postgis",
    "libcunit1-dev",
    "postgresql-server-dev-9.1",
    "postgresql-9.1-postgis"
  ];

  if Dir.glob("#{File.dirname(__FILE__)}/.vagrant/machines/default/*/id").empty?
	  pkg_cmd = ""

	  pkg_cmd << "apt-get update -qq; apt-get install -q -y python-software-properties; "

	  if ppaRepos.length > 0
		  ppaRepos.each { |repo| pkg_cmd << "add-apt-repository -y " << repo << " ; " }
		  pkg_cmd << "apt-get update -qq; "
	  end

	  # install packages we need we need
	  pkg_cmd << "apt-get install -q -y " + packageList.join(" ") << " ; "
	  config.vm.provision :shell, :inline => pkg_cmd
    scripts = [
      "startup.sh",
  	  "libgeotiff.sh",
      "nitro.sh",
      "hexer.sh",
      "p2g.sh",
      "soci.sh",
      "laszip.sh",
      "pdal.sh",
      "pgpointcloud.sh"
    ];
    scripts.each { |script| config.vm.provision :shell, :path => "scripts/vagrant/" << script }
  end
end
