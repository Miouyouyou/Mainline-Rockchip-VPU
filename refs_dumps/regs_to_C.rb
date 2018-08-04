#!/bin/ruby

require "fileutils"

MAX_NUMBERS_PER_LINE = 4

if ARGV.length != 1
	puts "./convert.rb file_number"
	puts "Example : ./convert.rb 0000"
	puts "In this example, mpp_dump_0000_regs is converted"
	abort
end

filename = "mpp_dump_#{ARGV[0].rjust(4, "0")}_regs"

unless File.exists?(filename)
	puts "#{filename} was not found :C"
	abort
end

if File.size(filename) < 4
	puts "#{filename} size should not be less than 4 bytes."
	puts "The expected size is 404 bytes"
	abort
end

regs = File.read(filename).unpack("I<*")

output = "int regs[#{regs.length}] {"

numbers_per_line = 0

regs.each {|int|
	output << "\n\t" if (numbers_per_line == 0)
	output << ("0x%08x, " % int)
	numbers_per_line += 1
	# We could warp power of 2 with &=
	# But let's avoid smart-ass technics on snippets
	numbers_per_line = 0 if numbers_per_line == MAX_NUMBERS_PER_LINE
}

output[output.rindex(","), 1] = ""

output << "\n};\n"

puts output
