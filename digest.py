#!/usr/bin/python

import sys

class Stream(object):

  def __init__(self, str):
    self.cursor = 0
    self.parts = str.split(" ")
  
  def has_more(self):
    return self.cursor < len(self.parts)
  
  def next(self):
    result = self.peek()
    self.cursor += 1
    return result
  
  def peek(self):
    return self.parts[self.cursor]

x = 0.0
y = 0.0
def main(args):
  assert len(args) == 1
  stream = Stream(open(args[0], "rt").read())
  def format_float(val):
    intpart = int(val)
    fracpart = val - intpart
    return "v(%s, %s)" % (intpart, fracpart)
  def parse_coord(coord):
    [x_str, y_str] = coord.split(",")
    return (float(x_str), float(y_str))
  def emit(is_rel, op, *args):
    global x
    global y
    def process_coord(str):
      (vx, vy) = parse_coord(str)
      if is_rel:
        vx += x
        vy += y
      return "%s, %s" % (format_float(vx), format_float(vy))
    print "  %s(%s)," % (op, ", ".join(map(process_coord, args)))
    (dx, dy) = parse_coord(args[-1])
    if is_rel:
      x += dx
      y += dy
    else:
      x = dx
      y = dy
  while stream.has_more():
    instr = stream.next()
    if instr == "m":
      emit(True, "move_to", stream.next())
      while stream.has_more() and len(stream.peek()) > 1:
        emit(True, "line_to", stream.next())
    elif instr == "M":
      emit(False, "move_to", stream.next())
      while stream.has_more() and len(stream.peek()) > 1:
        emit(False, "line_to", stream.next())
    elif instr == "L":
      while stream.has_more() and len(stream.peek()) > 1:
        emit(False, "line_to", stream.next())
    elif instr == "l":
      while stream.has_more() and len(stream.peek()) > 1:
        emit(True, "line_to", stream.next())
    elif instr == "c":
      while stream.has_more() and len(stream.peek()) > 1:
        emit(True, "curve_to", stream.next(), stream.next(), stream.next())
    elif instr == "C":
      while stream.has_more() and len(stream.peek()) > 1:
        emit(False, "curve_to", stream.next(), stream.next(), stream.next())
    else:
      print instr
  print "  end()"

if __name__ == "__main__":
  main(sys.argv[1:])
