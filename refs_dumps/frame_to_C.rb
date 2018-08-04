#!/bin/ruby

require "fileutils"

MAX_NUMBERS_PER_LINE = 8

if ARGV.length != 1
	puts "./convert.rb file_number"
	puts "Example : ./convert.rb 0000"
	puts "In this example, mpp_dump_0000_fram is converted"
	abort
end

filename = "mpp_dump_#{ARGV[0].rjust(4, "0")}_fram"

unless File.exists?(filename)
	puts "#{filename} was not found :C"
	abort
end

if File.size(filename) < 4
	puts "#{filename} size should not be less than 4 bytes."
	puts "The expected size is 404 bytes"
	abort
end

encoded_frame = File.read(filename).unpack("C*")

output = "uint8_t encoded_frame[#{encoded_frame.length}] {"

numbers_per_line = 0

encoded_frame.each {|octet|
	output << "\n\t" if (numbers_per_line == 0)
	output << ("0x%02x, " % octet)
	numbers_per_line += 1
	# We could warp power of 2 with &=
	# But let's avoid smart-ass technics for now
	numbers_per_line = 0 if numbers_per_line == MAX_NUMBERS_PER_LINE
}

output[output.rindex(","), 1] = ""

output << "\n};\n"

puts output
