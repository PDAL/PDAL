# -*- mode: ruby -*-
# vi: set ft=ruby :

require 'socket'
require 'ipaddr'

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"


# Evaluate a block passing one argument, an integer plucked from an environment
# variable. If that integer is zero, or the environment variable evaluates to
# zero with String#to_i, then don't evaluate the block.
def with_nonzero_integer_envvar(envvar, default = 0)
  integer = ENV[envvar] ? ENV[envvar].to_i : default
  if integer == 0
    # noop
  else
    yield integer
  end
end


Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "trusty64"

  config.vm.hostname = "pdal-vagrant"
  config.vm.box_url = "https://vagrantcloud.com/ubuntu/trusty64/version/1/provider/virtualbox.box"
  config.vm.host_name = "pdal-vagrant"

  # Set the bash environment variable PDAL_VAGRANT_SSH_FORWARD_AGENT to any
  # value to turn on ssh forwarding. This allows you to use your host machine's
  # ssh credentials inside your guest box, for example when interacting with
  # private github repositories.
  #
  # To confirm that ssh fowarding is working, run the following from inside the
  # guest machine:
  #
  #   ssh -T git@github.com
  #
  # You should see something like "Hi <your name here>! You've successfully
  # authenticated, but GitHub does not provide shell access."
  #
  # You may need to run `ssh-add` on your host machine to add your private key
  # identities to the authentication agent.
  if ENV['PDAL_VAGRANT_SSH_FORWARD_AGENT']
    config.ssh.forward_agent = true
  end

  # Set PDAL_VAGRANT_PORT_80_FORWARD to customize the target port
  # for the guest port 80. To disable guest port 80 forwarding, set
  # PDAL_VAGRANT_PORT_80_FORWARD to any value that cannot be parsed to
  # an integer with ruby's String#to_i method (e.g. 'false').
  with_nonzero_integer_envvar('PDAL_VAGRANT_PORT_80_FORWARD', 8080) do |host_port|
    config.vm.network :forwarded_port, guest: 80, host: host_port
  end

  config.vm.provider :virtualbox do |vb|
    # Set PDAL_VAGRANT_VIRTUALBOX_MEMORY to customize the virtualbox vm memory
    with_nonzero_integer_envvar('PDAL_VAGRANT_VIRTUALBOX_MEMORY', 4096) do |memory|
      vb.customize ["modifyvm", :id, "--memory", memory]
    end
    # Set PDAL_VAGRANT_VIRTUALBOX_CPUS to customize the virtualbox vm cpus
    with_nonzero_integer_envvar('PDAL_VAGRANT_VIRTUALBOX_CPUS', 2) do |cpus|
      vb.customize ["modifyvm", :id, "--cpus", cpus]
    end
    # Set PDAL_VAGRANT_VIRTUALBOX_IOAPIC to customize the virtualbox vm ioapic
    vb.customize ["modifyvm", :id, "--ioapic", ENV['PDAL_VAGRANT_VIRTUALBOX_IOAPIC'] || "on"]
    vb.name = "pdal-vagrant"

    # Set PDAL_VAGRANT_VIRTUALBOX_ENABLE_GUI to turn on the gui
    if ENV['PDAL_VAGRANT_VIRTUALBOX_ENABLE_GUI']
      vb.gui = true
    end
  end


  if RUBY_PLATFORM.include? "darwin"
    # If on a Mac, set PDAL_VAGRANT_PRIVATE_NETWORK_IP to customize
    # the private network's IP. Set to a non-IP value to disable private networking.
    if ENV['PDAL_VAGRANT_PRIVATE_NETWORK_IP']
      begin
        ipaddr = IPAddr.new ENV['PDAL_VAGRANT_PRIVATE_NETWORK_IP']
      rescue ArgumentError
        # noop
      else
        config.vm.network "private_network", ip: ipaddr
      end
    else
      config.vm.network "private_network", ip: "192.168.50.4"
    end

    # If on a Mac, set PDAL_VAGRANT_DISABLE_NFS to false to disable nfs mounting
    use_nfs = !ENV['PDAL_VAGRANT_DISABLE_NFS']
    config.vm.synced_folder ".", "/vagrant", nfs: use_nfs

    if Socket.gethostname.include? "pyro" # Howard's machine
      config.vm.synced_folder "/Users/hobu/dev/git/pointcloud", "/pointcloud", nfs: use_nfs
    end
  end

  if RUBY_PLATFORM.include? "win32"
    config.vm.synced_folder ".", "/vagrant", type: "smb"
  end

  ppaRepos = [
    "ppa:ubuntugis/ubuntugis-unstable",
    "ppa:boost-latest/ppa"
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
    "libboost-all-dev",
    "libbz2-dev",
    "libsqlite0-dev",
    "cmake-curses-gui",
    "screen",
    "postgis",
    "libcunit1-dev",
    "postgresql-server-dev-9.3",
    "postgresql-9.3-postgis-2.1",
    "libmsgpack-dev",
    "libgeos++-dev",
    "vim",
    "libeigen3-dev",
    "libflann-dev",
    "libglew-dev"
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
      "pcl.sh",
      "pdal.sh",
      "pgpointcloud.sh"
    ];
    scripts.each { |script| config.vm.provision :shell, :path => "scripts/vagrant/" << script }
  end
end
