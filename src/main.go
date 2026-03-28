package main

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

// Data types

type Vec3 struct{ X, Y, Z float64 }

type Triangle struct{ A, B, C Vec3 }

type AABB struct {
	Min, Max Vec3
}

func (b AABB) Center() Vec3 {
	return Vec3{
		(b.Min.X + b.Max.X) / 2,
		(b.Min.Y + b.Max.Y) / 2,
		(b.Min.Z + b.Max.Z) / 2,
	}
}

func (b AABB) Size() Vec3 {
	return Vec3{b.Max.X - b.Min.X, b.Max.Y - b.Min.Y, b.Max.Z - b.Min.Z}
}

// OBJ parsing

func parseOBJ(path string) ([]Vec3, []Triangle, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, nil, fmt.Errorf("Gagal Buka File: %w", err)
	}
	defer f.Close()

	var vertices []Vec3
	var triangles []Triangle
	lineNum := 0

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		lineNum++
		line := strings.TrimSpace(scanner.Text())
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}

		switch fields[0] {
		case "v":
			if len(fields) < 4 {
				return nil, nil, fmt.Errorf("invalid vertex on line %d: %q", lineNum, line)
			}
			x, err1 := strconv.ParseFloat(fields[1], 64)
			y, err2 := strconv.ParseFloat(fields[2], 64)
			z, err3 := strconv.ParseFloat(fields[3], 64)
			if err1 != nil || err2 != nil || err3 != nil {
				return nil, nil, fmt.Errorf("invalid vertex coordinates on line %d: %q", lineNum, line)
			}
			vertices = append(vertices, Vec3{x, y, z})

		case "f":
			if len(fields) < 4 {
				return nil, nil, fmt.Errorf("invalid face on line %d (need ≥3 indices): %q", lineNum, line)
			}
			//  Mengolah tiap index
			//	format face hanya sebagai integer index
			indices := make([]int, 0, len(fields)-1)
			for _, tok := range fields[1:] {
				//  Ambil bagian pertama saja
				parts := strings.Split(tok, "/")
				idx, err := strconv.Atoi(parts[0])
				if err != nil {
					return nil, nil, fmt.Errorf("invalid face index on line %d: %q", lineNum, tok)
				}
				if idx < 1 || idx > len(vertices) {
					return nil, nil, fmt.Errorf("face index %d out of range on line %d", idx, lineNum)
				}
				indices = append(indices, idx-1) 
			}
			//  
			for i := 1; i < len(indices)-1; i++ {
				triangles = append(triangles, Triangle{
					A: vertices[indices[0]],
					B: vertices[indices[i]],
					C: vertices[indices[i+1]],
				})
			}

		default:
		}
	}

	if err := scanner.Err(); err != nil {
		return nil, nil, err
	}
	if len(vertices) == 0 {
		return nil, nil, fmt.Errorf("File OBJ tidak valid, tidak ada vertex")
	}
	if len(triangles) == 0 {
		return nil, nil, fmt.Errorf("File OBJ tidak falid, tidak ada face")
	}
	return vertices, triangles, nil
}

// Bounding box

func computeBounds(vertices []Vec3) AABB {
	mn := Vec3{math.MaxFloat64, math.MaxFloat64, math.MaxFloat64}
	mx := Vec3{-math.MaxFloat64, -math.MaxFloat64, -math.MaxFloat64}
	for _, v := range vertices {
		if v.X < mn.X { mn.X = v.X }
		if v.Y < mn.Y { mn.Y = v.Y }
		if v.Z < mn.Z { mn.Z = v.Z }
		if v.X > mx.X { mx.X = v.X }
		if v.Y > mx.Y { mx.Y = v.Y }
		if v.Z > mx.Z { mx.Z = v.Z }
	}

	// Vektor diubah menjadi kubus
	size := mn
	_ = size
	sx := mx.X - mn.X
	sy := mx.Y - mn.Y
	sz := mx.Z - mn.Z
	side := math.Max(sx, math.Max(sy, sz))

	//  Vektor ditengahkan dan dilebarkan
	cx := (mn.X + mx.X) / 2
	cy := (mn.Y + mx.Y) / 2
	cz := (mn.Z + mx.Z) / 2
	half := side / 2 * 1.001 // tambahan
	return AABB{
		Min: Vec3{cx - half, cy - half, cz - half},
		Max: Vec3{cx + half, cy + half, cz + half},
	}
}

// Triangle–AABB intersection (SAT)

func triAABBIntersect(tri Triangle, box AABB) bool {
	center := box.Center()
	half := Vec3{
		(box.Max.X - box.Min.X) / 2,
		(box.Max.Y - box.Min.Y) / 2,
		(box.Max.Z - box.Min.Z) / 2,
	}

	//  Mengubah segitiga menjadi kotak di tengah
	v0 := Vec3{tri.A.X - center.X, tri.A.Y - center.Y, tri.A.Z - center.Z}
	v1 := Vec3{tri.B.X - center.X, tri.B.Y - center.Y, tri.B.Z - center.Z}
	v2 := Vec3{tri.C.X - center.X, tri.C.Y - center.Y, tri.C.Z - center.Z}

	//  Menyatakan edges
	e0 := Vec3{v1.X - v0.X, v1.Y - v0.Y, v1.Z - v0.Z}
	e1 := Vec3{v2.X - v1.X, v2.Y - v1.Y, v2.Z - v1.Z}
	e2 := Vec3{v0.X - v2.X, v0.Y - v2.Y, v0.Z - v2.Z}

	//  SAT: Menguji cross-product vektor dan edges
	axes := [][2]Vec3{
		{{1, 0, 0}, e0}, {{0, 1, 0}, e0}, {{0, 0, 1}, e0},
		{{1, 0, 0}, e1}, {{0, 1, 0}, e1}, {{0, 0, 1}, e1},
		{{1, 0, 0}, e2}, {{0, 1, 0}, e2}, {{0, 0, 1}, e2},
	}
	for _, pair := range axes {
		a := cross(pair[0], pair[1])
		p0 := dot(a, v0)
		p1 := dot(a, v1)
		p2 := dot(a, v2)
		r := half.X*math.Abs(a.X) + half.Y*math.Abs(a.Y) + half.Z*math.Abs(a.Z)
		mn := min3(p0, p1, p2)
		mx := max3(p0, p1, p2)
		if mn > r || mx < -r {
			return false
		}
	}

	//  SAT: 3 AABB face normals
	if max3(v0.X, v1.X, v2.X) < -half.X || min3(v0.X, v1.X, v2.X) > half.X { return false }
	if max3(v0.Y, v1.Y, v2.Y) < -half.Y || min3(v0.Y, v1.Y, v2.Y) > half.Y { return false }
	if max3(v0.Z, v1.Z, v2.Z) < -half.Z || min3(v0.Z, v1.Z, v2.Z) > half.Z { return false }

	//  SAT: triangle face normal
	normal := cross(e0, e1)
	d := dot(normal, v0)
	r2 := half.X*math.Abs(normal.X) + half.Y*math.Abs(normal.Y) + half.Z*math.Abs(normal.Z)
	if d > r2 || d < -r2 { return false }

	return true
}

func cross(a, b Vec3) Vec3 {
	return Vec3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}
func dot(a, b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }
func min3(a, b, c float64) float64 { return math.Min(a, math.Min(b, c)) }
func max3(a, b, c float64) float64 { return math.Max(a, math.Max(b, c)) }

// Octree

type OctreeNode struct {
	Bounds   AABB
	Children [8]*OctreeNode
	IsLeaf   bool
	IsFilled bool //  Leaf yang saling bertepatan menjadi voxel
}

//  Stats
type BuildStats struct {
	NodesPerDepth   []int
	PrunedPerDepth  []int
	MaxDepth        int
}

func buildOctree(bounds AABB, triangles []Triangle, maxDepth int) (*OctreeNode, BuildStats) {
	stats := BuildStats{
		NodesPerDepth:  make([]int, maxDepth+1),
		PrunedPerDepth: make([]int, maxDepth+1),
		MaxDepth:       maxDepth,
	}
	root := &OctreeNode{Bounds: bounds}
	subdivide(root, triangles, 0, maxDepth, &stats)
	return root, stats
}

func subdivide(node *OctreeNode, triangles []Triangle, depth, maxDepth int, stats *BuildStats) {
	stats.NodesPerDepth[depth]++

	//  Menyaring segitiga yang berlintasan
	var hitting []Triangle
	for _, tri := range triangles {
		if triAABBIntersect(tri, node.Bounds) {
			hitting = append(hitting, tri)
		}
	}

	if len(hitting) == 0 {
		//  Buntu
		stats.PrunedPerDepth[depth]++
		node.IsLeaf = true
		node.IsFilled = false
		return
	}

	if depth == maxDepth {
		// Kedalaman sudah maksimal
		node.IsLeaf = true
		node.IsFilled = true
		return
	}

	//  Bagi menjadi 8 subdivisi
	c := node.Bounds.Center()
	mn := node.Bounds.Min
	mx := node.Bounds.Max

	subBounds := [8]AABB{
		{Vec3{mn.X, mn.Y, mn.Z}, Vec3{c.X, c.Y, c.Z}},
		{Vec3{c.X, mn.Y, mn.Z}, Vec3{mx.X, c.Y, c.Z}},
		{Vec3{mn.X, c.Y, mn.Z}, Vec3{c.X, mx.Y, c.Z}},
		{Vec3{c.X, c.Y, mn.Z}, Vec3{mx.X, mx.Y, c.Z}},
		{Vec3{mn.X, mn.Y, c.Z}, Vec3{c.X, c.Y, mx.Z}},
		{Vec3{c.X, mn.Y, c.Z}, Vec3{mx.X, c.Y, mx.Z}},
		{Vec3{mn.X, c.Y, c.Z}, Vec3{c.X, mx.Y, mx.Z}},
		{Vec3{c.X, c.Y, c.Z}, Vec3{mx.X, mx.Y, mx.Z}},
	}

	for i := 0; i < 8; i++ {
		child := &OctreeNode{Bounds: subBounds[i]}
		node.Children[i] = child
		subdivide(child, hitting, depth+1, maxDepth, stats)
	}
}

//  Mengumpulkan semua leaf nodes (voxels)
func collectVoxels(node *OctreeNode) []AABB {
	if node == nil {
		return nil
	}
	if node.IsLeaf {
		if node.IsFilled {
			return []AABB{node.Bounds}
		}
		return nil
	}
	var result []AABB
	for _, child := range node.Children {
		result = append(result, collectVoxels(child)...)
	}
	return result
}

// OBJ output

func writeVoxelOBJ(path string, voxels []AABB) (int, int, error) {
	f, err := os.Create(path)
	if err != nil {
		return 0, 0, err
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	fmt.Fprintf(w, "# Voxelized OBJ — %d voxels\n", len(voxels))

	vertexOffset := 0
	totalFaces := 0

	//  Mengiterasi tiap voxel yang memiliki 8 vertex dan 6 faces
	for _, box := range voxels {
		mn := box.Min
		mx := box.Max

		//  8 vertex
		corners := [8]Vec3{
			{mn.X, mn.Y, mn.Z}, {mx.X, mn.Y, mn.Z},
			{mx.X, mx.Y, mn.Z}, {mn.X, mx.Y, mn.Z},
			{mn.X, mn.Y, mx.Z}, {mx.X, mn.Y, mx.Z},
			{mx.X, mx.Y, mx.Z}, {mn.X, mx.Y, mx.Z},
		}
		for _, v := range corners {
			fmt.Fprintf(w, "v %f %f %f\n", v.X, v.Y, v.Z)
		}

		//  6 faces as quads (i+offset, 1-based)
		o := vertexOffset
		faces := [6][4]int{
			{o + 1, o + 2, o + 3, o + 4}, //  bottom  (-Y)
			{o + 5, o + 8, o + 7, o + 6}, //  top     (+Y)
			{o + 1, o + 5, o + 6, o + 2}, //  front   (-Z)
			{o + 4, o + 3, o + 7, o + 8}, //  back    (+Z)
			{o + 1, o + 4, o + 8, o + 5}, //  left    (-X)
			{o + 2, o + 6, o + 7, o + 3}, //  right   (+X)
		}
		for _, face := range faces {
			fmt.Fprintf(w, "f %d %d %d %d\n", face[0], face[1], face[2], face[3])
			totalFaces++
		}

		vertexOffset += 8
	}

	return vertexOffset, totalFaces, w.Flush()
}

// main

func main() {
	if len(os.Args) < 3 {
		fmt.Fprintln(os.Stderr, "Usage: voxelizer <input.obj> <max_depth>")
		os.Exit(1)
	}

	inputPath := os.Args[1]
	maxDepth, err := strconv.Atoi(os.Args[2])
	if err != nil || maxDepth < 1 {
		fmt.Fprintln(os.Stderr, "Integer maxDepth harus positif")
		os.Exit(1)
	}

	start := time.Now()

	//  Parse
	vertices, triangles, err := parseOBJ(inputPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error parsing OBJ: %v\n", err)
		os.Exit(1)
	}

	//  Octree Build
	bounds := computeBounds(vertices)
	root, stats := buildOctree(bounds, triangles, maxDepth)

	//  Collect Voxels
	voxels := collectVoxels(root)

	//  Print Output
	ext := filepath.Ext(inputPath)
	base := strings.TrimSuffix(inputPath, ext)
	outputPath := fmt.Sprintf("%s_voxelized_depth%d.obj", base, maxDepth)

	totalVertices, totalFaces, err := writeVoxelOBJ(outputPath, voxels)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Gagal Print OBJ: %v\n", err)
		os.Exit(1)
	}

	elapsed := time.Since(start)

	//  ── CLI stats ──────────────────────────────
	fmt.Printf("Voxels (filled leaf nodes) : %d\n", len(voxels))
	fmt.Printf("Vertices in output         : %d\n", totalVertices)
	fmt.Printf("Faces in output            : %d\n", totalFaces)
	fmt.Println()
	fmt.Println("Octree node stats (nodes built per depth):")
	for d := 0; d <= maxDepth; d++ {
		if stats.NodesPerDepth[d] > 0 {
			fmt.Printf("  %d : %d\n", d+1, stats.NodesPerDepth[d])
		}
	}
	fmt.Println()
	fmt.Println("Nodes pruned (no geometry) per depth:")
	for d := 0; d <= maxDepth; d++ {
		if stats.NodesPerDepth[d] > 0 {
			fmt.Printf("  %d : %d\n", d+1, stats.PrunedPerDepth[d])
		}
	}
	fmt.Println()
	fmt.Printf("Octree depth               : %d\n", maxDepth)
	fmt.Printf("Time elapsed               : %v\n", elapsed)
	fmt.Printf("Output file                : %s\n", outputPath)
}
