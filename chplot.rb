require 'hotcocoa/graphics'
require 'optparse'

include HotCocoa
include Graphics

options = {}

optparse = OptionParser.new do|opts|
  # Set a banner, displayed at the top
  # of the help screen.
  opts.banner = "Usage: flowin.rb [options] inputfile outputfile"

  # Define the options, and what they do
  options[:type] = :image
  opts.on( '-p', '--pdf', 'Create a PDF File' ) do
    options[:type] = :pdf
  end

  options[:width] = 600
  options[:height] = 400

  opts.on( '-s', '--size WIDTHxHEIGHT', 'Image size' ) do |size|
    size = size.split("x")
    options[:width] = size[0].to_i
    options[:height] = size[1].to_i
  end

  # This displays the help screen, all programs are
  # assumed to have this option.
  opts.on( '-h', '--help', 'Display this screen' ) do
    puts opts
    exit
  end
end

# Parse the cmd line options
optparse.parse!

# Set the outfile filename
INFILE = ARGV[0]
INFILE = "in.chplot" if INFILE == nil
options[:flowfile] = INFILE



if ARGV[1] == nil
  OUTFILE = INFILE.split(".")[0] + ".png" if options[:type] == :image
  OUTFILE = INFILE.split(".")[0] + ".pdf" if options[:type] == :pdf
else
  OUTFILE = ARGV[1]
end
options[:filename] = OUTFILE

# A little complex number hack

class Numeric
  def i
    Complex(0,self)
  end
end

class Complex
  def apply(map)
    map.apply(self)
  end
  
  def normal
    self.imag - self.real.i
  end
  
  def <=> other
    if self.real != other.real then
      return self.real <=> other.real
    else
      return self.abs() <=> other.abs()
    end
  end
  
  def unify_at(z)
    l = self.abs() / z.imag
    return self / l
  end
end

class Invariant
  attr_reader :points
  
  def initialize(a,b,c)
    @points = [a,b,c]
  end
  
  def apply(map)
    result = Invariant.new(*@points.map{|z| map.apply(z)})
  end
  
  def center_point(epsilon)
    a = @points[0]
    b = @points[1]
    c = @points[2]
  
    delta1  = b-a
    delta2  = c-b
    
    # Swap points if needed
    if delta1.real == 0 then
      b = @points[0]
      a = @points[1]
      c = @points[2]
      
      delta1  = b-a
      delta2  = c-b
    elsif delta2.real == 0 then
      a = @points[0]
      c = @points[1]
      b = @points[2]
    end
    
    delta1  = b-a
    delta2  = c-b
        
    if delta2.real.abs < epsilon/100.0 and delta1.real.abs < epsilon/100.0 then
      m2 = m1 = 0
    else
      m1 = (delta1.imag.to_f)/(delta1.real.to_f)
      m2 = (delta2.imag.to_f)/(delta2.real.to_f)
    end
    
    if m1 == 0 then
      m1,m2 = m2,m1
      a,b,c = c,b,a
    end

    if (m2-m1).abs < epsilon
      return MoebiusMap::INFINITY
    else
      x = (m1*m2 * (c.imag-a.imag) + m1 * (b.real + c.real) - m2 * (a.real + b.real)) / (2 * (m1-m2).to_f)
      y = -1/m1.to_f*(x - (a.real+b.real)/2.0) +  (a.imag+b.imag)/2.0
      center = x+y.i
    end
  end
  
    
  def path_in_canvas(canvas, from = nil, to = nil, extending_segment = 0, path_given = nil)
    if path_given then
      path = path_given
    else
      path = Path.new
    end
    
    center = center_point(canvas.resolution * 0.2)
    
    if center == MoebiusMap::INFINITY
      # It's a line
      
      base  = @points[0]
      head  = @points[1]
      
      delta = head-base
      delta /= delta.abs()
      
      offset_im = base.imag-canvas.c_im_min
      offset_re = base.real-canvas.c_real_min
    
      imbase = base - offset_im * 1/delta.imag.to_f * delta
      rebase = base - offset_re * 1/delta.real.to_f * delta
    
      if imbase.real < canvas.c_real_min then
        base = rebase
        head = base + (canvas.c_real_max - canvas.c_real_min) *  1/delta.real.to_f * delta 
      else
        base = imbase
        head = base + (canvas.c_im_max - canvas.c_im_min) *  1/delta.imag.to_f * delta 
      end
      
      
      base = from if from && extending_segment >= 0
      head = to if from && extending_segment <= 0
      
      
      
      path.moveto(*canvas.convert_complex_to_canvas(base)) if not path_given
      path.lineto(*canvas.convert_complex_to_canvas(head))
    else
      r = (center-@points[0]).abs()
       
      base = center - (r+r.i)
      head = center + (r+r.i)
      
      if from && to then
        theta_start = (from-center).arg if from
        theta_end   = (to-center).arg if to
        
        if theta_start < theta_end
          step = 2*Math::PI/ 360.0 
          
          if extending_segment < 0
            theta_start = 0
          elsif extending_segment > 0
            theta_end = Math::PI
          end
        else
          step = -2*Math::PI/ 360.0 
          if extending_segment < 0
            theta_start = Math::PI
          elsif extending_segment > 0
            theta_end = 0
          end
        end
        z = center + Complex.polar(r, theta_start)
        path.moveto(*canvas.convert_complex_to_canvas(z)) if not path_given
        
        theta_start.step(theta_end, step) do |theta|
          z1 = center + Complex.polar(r, theta)
          x1,y1 = *canvas.convert_complex_to_canvas(z1)
          path.lineto(x1,y1)
        end
        
        z1 = center + Complex.polar(r, theta_end)
        x1,y1 = *canvas.convert_complex_to_canvas(z1)
        path.lineto(x1,y1)
      else
        
        x1,y1 = canvas.convert_complex_to_canvas(base)
        x2,y2 = canvas.convert_complex_to_canvas(head)
        path.oval(x1,y1, x2-x1, y2-y1)
        
      end
    end
    path
  end
end

class Geodesic < Invariant
  
  def initialize(a,b)
    secant  = (b-a)
    if secant.real.abs < 0.0000001 then
      return super(a,b,b+1.i)
    end
    
    normal  = -secant.imag + secant.real.i
    c       = a+secant/2.0
    
    if normal.real.abs < 0.000001 then
      centre = c.real + 0.i
    else
      magnitude = normal.imag/normal.real.to_f
      centre = c.real + c.imag.to_f * -1/magnitude + 0.i
    end
  
    radius = (a-centre).abs()
    
    c1 = centre - radius
    c2 = centre + radius
    
    dist1 = [(a-c1).abs(),(b-c1).abs()].min
    dist2 = [(a-c2).abs(),(b-c2).abs()].min
    
    if dist2 > dist1
      super(a,b,c2)
    else
      super(a,b,c1)
    end
  end
end

class GeodesicSegment < Geodesic
  attr_reader :head
  attr_reader :tail
  
  def initialize(head,tail)
    super(head,tail)
    @head = head
    @tail = tail
  end
  
  def apply(map)
    head = map.apply(@head)
    tail = map.apply(@tail)
    result = GeodesicSegment.new(head, tail)
  end
  
  def path_in_canvas(canvas, extending_segment = 0, path_given = nil)
    super(canvas, @head, @tail, extending_segment, path_given)
  end
  
  def to_s
    "Geodesic segment from: #{@head} to: #{@tail}"
  end
end

class PointSet
  attr_reader :points
  attr_reader :derivatives
  
  def dotted_path_in_canvas(canvas, size = 1, style = :circle)
    path = Path.new
    @points.each do |point|
      skip if point == "INF"
      x1,y1 = canvas.convert_complex_to_canvas(point)
      x1 -= size/2.0
      y1 -= size/2.0
      
      if style == :circle
        path.oval(x1,y1, size, size)
      else
        path.moveto(x1,y1)
        path.lineto(x1+size,y1+size)
        path.moveto(x1,y1+size)
        path.lineto(x1+size,y1)
      end
    end
    path
  end
  
  def path_in_canvas(canvas, closed=false)
    path = Path.new
    start = @points.shift
    path.moveto(*canvas.convert_complex_to_canvas(start))
    @points.each do |point|
      path.lineto(*canvas.convert_complex_to_canvas(point))
    end
    path.lineto(*canvas.convert_complex_to_canvas(start)) if closed
    path
  end
  
  def tangent_arrow_at_time(time, canvas, invert = false)
    index = (time*@points.size).floor
    point = @points[index]
    if(index < @points.size)
      b = @points[index+1]
    else
      b = @points[index]
    end
    
    if(index > 0)
      a = @points[index-1]
    else
      a = @points[index]
    end
    
    dir = b-a
    dir = -dir if(invert)
    canvas.arrow_in_direction_path(point, dir)
  end
  
  def normal_arrow_at_time(time, canvas, invert = false)
    index = (time*@points.size).floor
    point = @points[index]
    if(index < @points.size)
      b = @points[index+1]
    else
      b = @points[index]
    end
    
    if(index > 0)
      a = @points[index-1]
    else
      a = @points[index]
    end
    
    dir = b-a
    dir = -dir if(invert)
    
    dir = Complex(dir.imag, -dir.real)
    
    canvas.arrow_in_direction_path(point, dir)
  end
  
  def initialize(points = [])
    @points = points
    @derivatives = Array.new(points.size)
  end
  
  def <<(point)
    @points << point unless point == @points[-1]
  end
  
  def add_derivative(proc)
    @derivatives << proc
  end
  
  def apply(map)
    result = PointSet.new()
    @points.each do |point| 
      result << map.apply(point)  
    end
    return result
  end
end

class GeodesicPolygon < PointSet
  def path_in_canvas(canvas)
    path = Path.new
    last = @points.shift
    first = last
    
    if first == MoebiusMap::INFINITY then
      x,y = canvas.convert_complex_to_canvas(@points[-1])
      y = canvas.height
      path.moveto(x,y)
    else
      path.moveto(*canvas.convert_complex_to_canvas(first))
    end
    @points.each do |point|
      extending = 0
      head = last
      tail = point
      if last == MoebiusMap::INFINITY then
        head = point + 10.i
        extending = 1
      elsif point == MoebiusMap::INFINITY then
        tail = last + 10.i
        extending = 1
      end
      segment = GeodesicSegment.new(head,tail)  
      segment.path_in_canvas(canvas,  extending, path)
      last = point
    end
    
    extending = 0
    if last == MoebiusMap::INFINITY then
      last = first + 10.i
      extending = 1
    elsif first == MoebiusMap::INFINITY then
      first = last + 10.i
      extending = -1
    end
    segment = GeodesicSegment.new(last,first)  
    segment.path_in_canvas(canvas, extending, path)
    path.endpath
    path
  end
  
  def apply(map)
    result = GeodesicPolygon.new()
    @points.each do |point| 
      result << map.apply(point)  
    end
    return result
  end
end

class Array
  def apply(map)
    self.map {|object| map[object]}
  end
end

class MoebiusMap
  
  include Comparable
  
  INFINITY = "INF"
  
  attr_reader :a, :b, :c, :d
  
  def initialize(a,b=nil,c=nil,d=nil)
    if b == nil then
      @a = a[0]
      @b = a[1]
      @c = a[2]
      @d = a[3]
    else
      @a = a
      @b = b
      @c = c
      @d = d
    end
  end
  
  def inverse
    @det = @a*@d-@b*@c
    MoebiusMap.new(@d/@det, -@b/@det, -@c/@det, @a/@det)
  end
  
  def apply(z)
    if z == INFINITY then
      return @a/@c
    end
    if @c*z+@d == 0 then
      return INFINITY
    end
    (@a*z+@b)/(@c*z+@d)
  end
  
  def derivative_at(z)
    return lambda { |xi| 1.0/(@c * z + @d)**2 * xi }
  end
  
  def [] (applicable)
    applicable.apply(self)
  end
  
  def *(other)
    case other
    when MoebiusMap
      MoebiusMap.new(@a*other.a+@b*other.c, @a*other.b+@b*other.d, @c*other.a+@d*other.c, @c*other.b+@d*other.d)
    when Flow
      return Flow.new { |t| (self * (other^t)).to_a }
    end
  end
  
  def to_a
    [@a,@b,@c,@d]
  end
  
  def to_s
    to_a.to_s
  end
  
  def <=> other
    to_a <=> other.to_a
  end
end

class Group
  def initialize(&block)
    @depth        = 5
    send(:instance_eval, &block)
    @generators.map! {|g| MoebiusMap.new(g)}
    @inverses     = @generators.map{|g| g.inverse}
    @base         = @generators + @inverses.reverse
  end
  
  def generators args
    return @generators if args.size == 0
    @generators       = args.values
    @generator_hash   = args
  end
  
  def commutators *args
  end
  
  def depth newdepth
    @depth = newdepth
  end
  
  def act_on (object, &block)
    dfs(object,0,block, nil, [])
  end
  
private 

  def dfs(object, curdepth, block, last = nil, path)
    if curdepth < @depth then
      @base.each_index do |i|
        next if i == last
        
        transformed = @base[i][object]
        block.call(transformed)       
        dfs(transformed, curdepth+1, block, @base.size-i-1)
      end
    end
  end
end

class Flow
  def initialize(&block)
    @flow_proc      = block
    @is_inside_proc = nil
    @generators     = nil
    @root           = nil
  end
  
  def constraint(&block)
    @is_inside_proc = block
  end
  
  def generators(*list)
    @generators = list
  end
  
  def ^ (t)
    return MoebiusMap.new(@flow_proc.call(t))
  end
  
  def [] z,  start = -1, stop = 1, step = 0.05, maxdepth = 5
    points  = PointSet.new
    
    curflow = self
    curz    = z
    toffset = 0
    
    start.step(stop,step) do |t|
      newpoint = (curflow^(t-toffset))[curz]
      newderiv = (curflow^(t-toffset)).derivative_at(curz)
      if @is_inside_proc
        if not @is_inside_proc.call(newpoint)
          puts "Not inside: #{newpoint}"
          transformer = MoebiusMap.new(1,0,0,1)
          retransformed, rederiv,transformer = dfs(newpoint,newderiv,transformer, 0, maxdepth, nil)
          if retransformed
            newpoint = retransformed
            newderiv = rederiv
            
            curz      = retransformed
            curflow   = transformer * curflow * transformer.inverse
            toffset   = t
            puts "Curz #{curz}"
          end
        end
      end
      
      points << newpoint
      points.add_derivative(newderiv)
    end
    points
  end
  
  def * object
    return Flow.new { |t| ((self^t) * object).to_a }
  end
    
  def dfs(z, derivative, transformer, depth, maxdepth, last_transform = nil)
    @generators.each_index do |i|
      
      # Avoid direct inversion
      
      if last_transform and @generators[i] != last_transform.inverse
        newz            = @generators[i][z]
        newderiv        = lambda{|x| @generators[i].derivative_at(z).call(derivative.call(x)) }
        newtransformer  = transformer * @generators[i] 
        return newz, newderiv, newtransformer if @is_inside_proc.call(newz)
        best, bestderiv, besttransformer = dfs(newz, newderiv, newtransformer, depth+1, maxdepth,@generators[i]) if depth < maxdepth 
        return best, bestderiv, besttransformer if best != nil
      end
      
      if @generators[i] !=last_transform
        newz = @generators[i].inverse[z]   
        newderiv  = lambda{|x| @generators[i].inverse.derivative_at(z).call(derivative.call(x)) }
        newtransformer  = transformer * @generators[i].inverse
        return newz, newderiv, newtransformer if @is_inside_proc.call(newz)
        best, bestderiv, besttransformer = dfs(newz, newderiv, newtransformer, depth+1, maxdepth,@generators[i].inverse) if depth < maxdepth
        return best, bestderiv, besttransformer if best != nil
      end
      
    end
    return nil
  end
end

class Pool
  attr_reader :cache
  attr_reader :objects
  
  def initialize
    @cache    = []
    @objects  = []
  end
  
  def flush
    @cache.each {|proc| proc.call }
  end
  
end

class ComplexCanvas < Canvas
  
  include Math  
  
  DefaultOptions = {:c_real_min => -1, :c_real_max => 1, :c_im_min => -1,  :c_im_max => 1}
  
  attr_reader :resolution
  
  attr_reader :ratio
  
  attr_reader :c_real_min
  attr_reader :c_real_max
  attr_reader :c_im_min
  attr_reader :c_im_max
  attr_reader :width
  attr_reader :height
  
  attr_reader :settings
  
  def initialize(options={}, &block)
    options = DefaultOptions.merge(options)
    
    @c_real_min   = options[:c_real_min]
    @c_real_max   = options[:c_real_max]
    @c_im_max     = options[:c_im_max]
    @c_im_min     = options[:c_im_min]
    
    @c_width      = @c_real_max-@c_real_min
    @c_height     = @c_im_max-@c_im_min
    
    @ratio        = @c_width/@c_height
    
    @settings     = {:point_style => :circle, :point_size => 3.0, :arrow_size => 0.5, :head_size => 1}
    @stack        = []
    
    
    @resolution = (@c_width/(options[:width].to_f))
    super(options, &block)
  end
  
  # Some general non graphical helpers
  def pi
    Math::PI
  end
  
  # Push and pop the graphics state
  def push
    @stack.push(@settings.clone)
    super
  end
  
  def pop
    settings = @stack.pop
    @settings = settings if settings
    super
  end
  
  def arrow_size(size)
    @settings[:arrow_size] = size
  end
  
  def head_size(head_size)
    @settings[:head_size] = head_size
  end
  
  def point_size(point_size)
    @settings[:point_size] = point_size
  end
  
  def point_style(style)
     @settings[:point_style] = style
   end
  
  # Point and Moebius map set creation
  
  def moebius_map(a,b=nil,c=nil,d=nil)
    return MoebiusMap.new(a,b,c,d)
  end
  
  def circle(center, radius)
    return Invariant.new(center+radius, center+radius.i, center-radius)
  end
  
  def line(base, direction)
    return Invariant.new(base, base+direction, base-direction)
  end
  
  def center(invariant)
    invariant.center_point(@resolution * 0.2)
  end
  
  def to_set(invariant, density = 360)
    c = center(invariant)
    r = (c - invariant.points[0]).abs()
    return circle_set(c, r, density)
  end
  
  def circle_set(center, radius, density = 360)
    circle      = PointSet.new
    angle_delta = 2*Math::PI/density
    angle       = 0
    
    (density+1).times do
      circle << center + radius*(Complex(Math.cos(angle),Math.sin(angle)))
      angle += angle_delta
    end
    return circle
  end
  
  def arc_set(center, radius, theta1, theta2, density = 360)
    circle      = PointSet.new
    angle_delta = 2*Math::PI/density
    angle       = theta1
    (density+1).times do
      circle << center + radius*(Complex(Math.sin(angle),Math.cos(angle)))
      break if angle > theta2
      angle += angle_delta
    end
    return circle
  end
  
  def segment_set(a,b, density = 360)
    segment = PointSet.new
    delta = (b-a)/density
    density.times do 
      segment << a
      a += delta
    end
    segment << a
    return segment
  end
  
  def flow(&block)
    return Flow.new(&block)
  end
  
  def group(&block)
    return Group.new(&block)
  end
  
  def stored_pool(&block)
    @pool = Pool.new
    block.call
    result_pool = @pool
    @pool=nil
    return result_pool
  end
  
  def pool(&block)
    @pool = Pool.new
    block.call
    @pool.flush
    @pool=nil
  end
    
  def pool_cache(object=nil, &block)
    if @pool == nil then
      block.call
      return false
    end
    
    case object[0]
    when :segment
      object[1].sort!
      @pool.objects.each do |other|
        delta1 = (other[1][0]-object[1][0]).abs()
        delta2 = (other[1][1]-object[1][1]).abs()
        return true if delta1+delta2 < 0.000000001
      end
    else
      return true if @pool.objects.include?(object)
    end
    @pool.objects << object
    @pool.cache << block
    
    return false
  end
  
  #  Core drawing methods
  def text(txt, z)
    pool_cache {
      x,y = convert_complex_to_canvas(z)
      super(txt,x,y)
    }
  end
  
  def draw_set(pointset, dotted = false, closed = false)
    pool_cache {
      if dotted then
        path = pointset.dotted_path_in_canvas(self, @settings[:point_size], @settings[:point_style])
      else
        path = pointset.path_in_canvas(self, closed)
      end
      draw_path(path)
    }
  end
  
  def draw_invariant(invariant)
    pool_cache {
      draw_path(invariant.path_in_canvas(self))
    }
  end
  
  def draw_point(z)
    pool_cache {
      return if z == "INF"
      x1,y1 = convert_complex_to_canvas(z)
      s = @settings[:point_size]
      if @settings[:point_style] == :circle
        x1 -= s/2.0
        y1 -= s/2.0
        oval(x1,y1, s, s)
      else
        x1 -= s/2.0
        y1 -= s/2.0
        lines([[x1,y1],[x1+s,y1+s]])
        lines([[x1,y1+s],[x1+s,y1]])
      end
    }
  end
  
  def arrow(base, head=nil)
    pool_cache {
      push
      nofill
      linejoin(:round)
      linecap(:round)
      if head
        draw_path(arrow_path(base,head))
      else
        draw_path(base)
      end
      pop
    }
  end
  
  def draw_pool(pool)
    pool.flush
  end
  
  # Drawing composition methods
  
  def draw(object)
    case object
    when Invariant
      draw_invariant(object)
    when Complex
      draw_point(object)
    when PointSet
      draw_set(object)
    when Pool
      draw_pool(object)
    end
  end
  
  # Draw axes & arrows
  def axes(xticks = 1, yticks = 1, subdivision = 10, noarrows = false, offset = 0)
    head_size = 0.7
    head_size = 0 if noarrows
    
    offset = @resolution * offset
    
    (@c_real_min.ceil).step(@c_real_max, xticks) do |i|
      cmoveto(i+offset.i)
      clineto(i+3*@resolution.i)
    end
    
    (@c_im_min.ceil).step(@c_im_max, yticks) do |i|
      cmoveto(i.i)
      clineto(i.i+3*@resolution)
    end
    
    (@c_real_min.ceil).step(@c_real_max, xticks/subdivision.to_f) do |i|
      cmoveto(i+offset.i)
      clineto(i+2*@resolution.i)
    end
    
    (@c_im_min.ceil).step(@c_im_max, yticks/subdivision.to_f) do |i|
      cmoveto(i.i)
      clineto(i.i+2*@resolution)
    end
    
    push
      head_size(head_size)
      
      arrow(@c_real_min+offset.i, @c_real_max-@resolution+offset.i)
      arrow(@c_im_min.i, (@c_im_max-@resolution).i)
    pop
  end
  
  def tangent_arrow(pointset,t = 0, invert = false)
    arrow(pointset.tangent_arrow_at_time(t, self, invert))
  end
  
  def normal_arrow(pointset,t = 0 , invert = false)
    arrow(pointset.normal_arrow_at_time(t, self, invert))
  end
    
  def arrow_in_direction(base, direction, size = nil)
    arrow(arrow_in_direction_path(base, direction, size))
  end
    
  # Conversion to local coordinates
  def convert_width_to_canvas(w)
    w*(@width/@c_width)
  end
  
  def convert_height_to_canvas(h)
    h*(@height/@c_height)
  end
  
  def convert_local_to_canvas(x,y,w,h)
    return (x-@c_real_min)*(@width/@c_width),(y-@c_im_min)*(@height/@c_height),w*(@width/@c_width),h*(@height/@c_height)
  end
  
  def convert_local_to_canvas(x,y)
    return (x-@c_real_min)*(@width/@c_width),(y-@c_im_min)*(@height/@c_height)
  end
  
  def convert_complex_to_canvas(z)
    if z==MoebiusMap::INFINITY then
      x= options[:width]/2
      y= options[:height] * 10000
      return x,y
    end
    convert_local_to_canvas(z.real, z.image)
  end
  
  # Helpers for drawing
  def cmoveto(z)
    moveto(*convert_complex_to_canvas(z))
  end

  def clineto(z)
    lineto(*convert_complex_to_canvas(z))
  end

  # Internal path creation

  def arrow_in_direction_path(base, direction, size = nil)
    direction /= direction.abs()
    size = @settings[:arrow_size] if size == nil
    arrow_path(base, base + direction* size)
  end

  def arrow_path(base, head)
    path = Path.new

    arrow_delta = (head-base)
    arrow_delta /= arrow_delta.abs()
    arrow_normal = Complex(arrow_delta.imag, -arrow_delta.real)

    # Determine the arrow head size
    size = 3 * @settings[:head_size] * @resolution

    cmoveto(base)
    clineto(head-(arrow_delta*1.5)*size)
    clineto(head-(arrow_delta*2+arrow_normal)*size)
    clineto(head)
    clineto(head-(arrow_delta*2-arrow_normal)*size)
    clineto(head-(arrow_delta*1.5)*size)
    path
  end
end

class HyperbolicCanvas < ComplexCanvas
  
  def initialize(options={}, &block)
    options[:c_im_min] = 0
    options[:c_im_max] = 2 if options[:c_im_max] == nil
    super(options, &block)
  end
  
  def axes(xticks = 1, yticks = 1, subdivision = 10, noarrows = false)
    super(xticks, yticks, subdivision, noarrows, 0.5)
  end
  
  def hyperbolic_distance(a,b)
    hyp_factor = ((a-b.conj()).abs()+(a-b).abs())/((a-b.conj()).abs()-(a-b).abs())
    Math.log(hyp_factor)
  end
  
  def geodesic(a,b)
    Geodesic.new(a,b)
  end
  
  def mapping_to_i(z, dir = nil)
    if dir==nil
      w     = z+1
    else
      dir /= dir.abs() / @resolution
      w     = z + dir
    end
    
    l       = geodesic(z,w)
    center  = l.center_point(@resolution*0.2)
    radius  = (center-l.points[0]).abs()
    xi_min  = center - radius
    xi_max  = center + radius
    return MoebiusMap.new(1,-xi_max,1,-xi_min)
  end
  
  def horocycle(a, dir, pos = true)
    normal = dir.imag - dir.real.i
    geodesic = geodesic_in_direction(a,normal)  
    center = geodesic.center_point(@resolution*0.2)
      
    if center == infinity
      dir /= dir.abs() / @resolution
      return line(a, dir)
    else    
      if pos 
        center = center+ (center-geodesic.points[0]).abs()
      else
        center = center- (center-geodesic.points[0]).abs()
      end
      return Invariant.new(a,center + @resolution, center)
    end
  end
  
  def horocycle_basepoint(a,dir, pos = true)
    normal = dir.imag - dir.real.i
    geodesic = geodesic_in_direction(a,normal)  
    center = geodesic.center_point(@resolution*0.2)
      
    if center == infinity
      return a.real = 0.i
    else    
      if pos 
        center = center+ (center-geodesic.points[0]).abs()
      else
        center = center- (center-geodesic.points[0]).abs()
      end
      return center
    end
  end
  
  def geodesic_in_direction(a, dir)
    dir /= dir.abs() / @resolution
    Geodesic.new(a,a+dir)
  end
  
  def geodesic_segment(head,tail)
    GeodesicSegment.new(head,tail)
  end
  
  def geodesic_polygon(points = [])
    GeodesicPolygon.new(points)
  end
  
  def draw_segment(segment, extending_segment = 0)
    pool_cache([:segment, [segment.head, segment.tail], extending_segment]) {
      draw_path(segment.path_in_canvas(self, extending_segment))
    }
  end
  
  def draw(object)
    case object
    when GeodesicSegment
      draw_segment(object)
    when GeodesicPolygon
      draw_path(object.path_in_canvas(self))
    else
      super(object)
    end
  end
  
  def infinity
    return MoebiusMap::INFINITY
  end
  
  def extend_positive
    1
  end
  
  def extend_negative
    -1
  end
  
end

class CHPlot
  def initialize(file,options)
    begin
      file    = File.new(file, "r")
    rescue
      puts "Couldn't find file"
      return
    end
    script  = file.read
    
    @hyperbolic_plane = false
    @options          = options
    @canvas           = nil
    begin
      self.instance_eval(script)
    rescue Exception => err
      puts "There is an ERROR in your description file \n #{err.message}"
      puts "#{err.backtrace}"
    end
    
    if @canvas
      @canvas.save
      `open #{OUTFILE}`
    end
  end
  
  def window(re_min = -1,re_max=1,im_min=-1,im_max=nil, keepratio=true)
    ratio = @options[:height]/@options[:width].to_f
    
    if keepratio && im_max != nil && (not (im_max == true || im_max == false))
      delta_im = im_max-im_min
      delta_re = re_max-re_min
      delta = delta_im - delta_re * ratio
      im_max -= delta/2
      im_min += delta/2
    end
    
    if keepratio && im_max == nil && (not (im_max == false))
      delta_im = im_min
      delta_re = re_max-re_min
      delta = delta_im - delta_re * ratio
      im_min -= delta
    end
    
    @options = @options.merge({:c_real_min => re_min, :c_real_max => re_max, :c_im_min => im_min,  :c_im_max => im_max})
  end
  
  def hyperbolic plane=nil
    @hyperbolic_plane = true
  end
  
  def plane
    nil
  end
  
  def graphics(&block)
    if @hyperbolic_plane then
      @options[:c_im_max] = @options[:c_im_min]
      @canvas = HyperbolicCanvas.new(@options, &block)
    else
      @canvas = ComplexCanvas.new(@options, &block)
    end
  end
end

# Load the definition file
plot = CHPlot.new(INFILE,options)