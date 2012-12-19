#!/usr/bin/perl

use File::Basename;

my $firmware_file;
my $firmware_version;
my $android_dir = "/data/local/tmp/";
my $bootloader = "mxt-app";
my $bootloader_location = "libs/armeabi/$bootloader";

################################################################################

=item adb_scan

Scans every second using adb for devices. If one device is found, try and
flash it, otherwise go back to waiting.

=cut

sub adb_scan
{
  print "Waiting for device...\n";

  while (true)
  {
    my @devices;

    # Detect device using adb
    my @devicesOutput = qx{adb devices};
    shift(@devicesOutput) =~ m/^List of devices/i or next;

    foreach (@devicesOutput) {
      chomp;
      next if m/^\s*$/;

      my ($serial, $desc) = split /\s+/, $_, 2;

      push @devices, { serial => $serial, desc => $desc };
    }

    if (@devices > 1) {
      print "ERROR: Multiple devices connected\n";
    }

    if (@devices == 1) {
      my $serial = $devices[0]->{serial};

      flash_device($serial);
    }

    sleep 1;
  }
}

################################################################################

=item adb_shell ( command )

Run a command using adb

=cut

sub adb_shell
{
  my $command = shift;
  my $result;

  $adb_command = 'adb shell ' . $command . ' 2>&1 |';

  unless (open (COMMAND, $adb_command))
  {
    print("open \"$adb_command\" failed " . $! . "\n");
    die;
  }

  while (<COMMAND>)
  {
    print($_);
    $result .= $_;
  }

  close(COMMAND);

  return $result;
}

################################################################################

=item print_rule

Print a line across the terminal

=cut

sub print_rule
{
  print '-' x 80 . "\n";
}


################################################################################

=item flash_device

=cut

sub flash_device
{
  my $device = shift;

  my $logdir = "device-$device";
  my $status_file = "$logdir/status.txt";

  print_rule;

  print "Detected device ", $device . "\n";

  if (! -e $logdir)
  {
    mkdir $logdir;
  }
  elsif (-e $status_file)
  {
    open (COMPLETED, "<", $status_file);
    my $completed = <COMPLETED>;
    if ($completed eq $firmware_version)
    {
      print "Device already flashed to firmware build $completed\n";
      print_rule;
      sleep 5;
      return;
    }
  }

  unlink($status_file);

  open (LOGFILE, ">>$logdir/log.txt");

  print "Capturing device information\n";
  adb_shell("dmesg > $logdir/dmesg.txt");
  adb_shell("su -c id > $logdir/id.txt");
  adb_shell("ls -l \"/dev/i2c*\" > $logdir/i2c.txt");
  adb_shell("ls -lR /sys/bus/i2c/drivers > $logdir/sysfs.txt");

  print "Uploading flash tool: ";
  system("adb push $bootloader_location $android_dir");

  print "Uploading firmware: ";
  system("adb push \"$firmware_file\" $android_dir");

  my $firmware_basename = basename($firmware_file);

  print "Running flash tool\n";
  system("adb logcat -c");
  adb_shell("su -c chmod 775 $android_dir/$bootloader");
  my $bootloader_output = adb_shell("su -c $android_dir/$bootloader --flash \"$android_dir/$firmware_basename\" --firmware-version $firmware_version");

  print LOGFILE $bootloader_output;

  print "Removing files\n";
  adb_shell("su -c rm -r $android_dir");

  print "Capturing log\n";
  system("adb logcat -d > $logdir/logcat.txt");

  if ($bootloader_output =~ m/SUCCESS/
      || $bootloader_output =~ m/Firmware already correct version/)
  {
    open (STATUS, ">", $status_file);
    print STATUS $firmware_version;
    close(STATUS);
  }

  if ($bootloader_output =~ m/SUCCESS/)
  {
    print "Restarting device\n";
    system("adb reboot");
  }


  print_rule;

  close(LOGFILE);
}

################################################################################

if ($#ARGV != 1) {
  print "usage: android-flash.pl <firmware_file> <firmware_version>\n";
  exit 1;
}

$firmware_file = $ARGV[0];
$firmware_version = $ARGV[1];

unless (qx{adb version} =~ m/^Android Debug Bridge/)
{
  print "Could not run adb - check PATH?\n";
  exit 1;
}


adb_scan();
