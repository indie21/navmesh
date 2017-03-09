package navmesh

import (
	"errors"
	"math/rand"

	. "github.com/spate/vectormath"
)

var (
	ERROR_TRIANGLELIST_ILLEGAL = errors.New("triangle list illegal")
)

type TriangleList struct {
	Vertices  []Point3
	Triangles [][3]int32 // triangles
}

type BorderList struct {
	Indices []int32 // 2pt as border
}

type Path struct {
	Line []Point3
}

type NavMesh struct{}

// 从P1处随机偏移一个到(min, max)的长度。
func spinOffset(p1, p2 Point3, min, max float32) Point3 {
	var v1 Vector3

	P3Sub(&v1, &p2, &p1)
	if v1.Length() < min {
		return p1
	}

	if v1.Length() < max {
		max = v1.Length()
	}

	V3Normalize(&v1, &v1)
	V3ScalarMul(&v1, &v1, rand.Float32()*(max-min)+min)
	V3AddP3(&v1, &v1, &p1)
	P3MakeFromV3(&p1, &v1)
	return p1
}

func (nm *NavMesh) Route(list TriangleList, start, end *Point3) (*Path, error) {
	r := Path{}
	// 计算临边
	border := nm.create_border(list.Triangles)
	// 目标点
	vertices := append(list.Vertices, *end)
	border = append(border, int32(len(vertices))-1, int32(len(vertices))-1)

	// 第一个可视区域
	line_start := start
	last_vis_left, last_vis_right, last_p_left, last_p_right := nm.update_vis(start, vertices, border, 0, 1)
	var res Vector3
	for k := 2; k <= len(border)-2; k += 2 {
		cur_vis_left, cur_vis_right, p_left, p_right := nm.update_vis(line_start, vertices, border, k, k+1)
		V3Cross(&res, last_vis_left, cur_vis_right)
		if res.Z > 0 { // 左拐点
			line_start = &vertices[border[last_p_left]]
			r.Line = append(r.Line, *line_start)
			// 找到一条不共点的边作为可视区域
			i := 2 * (last_p_left/2 + 1)
			for ; i <= len(border)-2; i += 2 {
				if border[last_p_left] != border[i] && border[last_p_left] != border[i+1] {
					last_vis_left, last_vis_right, last_p_left, last_p_right = nm.update_vis(line_start, vertices, border, i, i+1)
					break
				}
			}

			k = i
			continue
		}

		V3Cross(&res, last_vis_right, cur_vis_left)
		if res.Z < 0 { // 右拐点
			line_start = &vertices[border[last_p_right]]
			r.Line = append(r.Line, *line_start)
			// 找到一条不共点的边
			i := 2 * (last_p_right/2 + 1)
			for ; i <= len(border)-2; i += 2 {
				if border[last_p_right] != border[i] && border[last_p_right] != border[i+1] {
					last_vis_left, last_vis_right, last_p_left, last_p_right = nm.update_vis(line_start, vertices, border, i, i+1)
					break
				}
			}

			k = i
			continue
		}

		V3Cross(&res, last_vis_left, cur_vis_left)
		if res.Z < 0 {
			last_vis_left = cur_vis_left
			last_p_left = p_left
		}

		V3Cross(&res, last_vis_right, cur_vis_right)
		if res.Z > 0 {
			last_vis_right = cur_vis_right
			last_p_right = p_right
		}
	}

	return &r, nil
}

func (nm *NavMesh) create_border(list [][3]int32) []int32 {
	var border []int32
	for k := 0; k < len(list)-1; k++ {
		for _, i := range list[k] {
			for _, j := range list[k+1] {
				if i == j {
					border = append(border, i)
				}
			}
		}
	}
	return border
}

// 按一定的比例随机收缩，可以让点有一定的变化.
func (nm *NavMesh) shrink_border(list []int32, vertices []Point3, rate float32) (nBorder []int32, nVertices []Point3) {
	nVertices = make([]Point3, len(list))
	nBorder = make([]int32, len(list))

	for k := 0; k < len(list); k += 2 {

		// if vertices[list[k]] == vertices[list[k+1]] {
		// 	nBorder[k] = int32(k)
		// 	nBorder[k+1] = int32(k + 1)
		// 	nVertices[k] = vertices[list[k]]
		// 	nVertices[k+1] = vertices[list[k+1]]
		// 	break
		// }

		//1.计算两点中点.
		//2.计算当前中点到上下两点的向量.
		//3.对向量大小进行减小后重新计算上线两点。
		mx := (vertices[list[k]].X + vertices[list[k+1]].X) / 2
		my := (vertices[list[k]].Y + vertices[list[k+1]].Y) / 2
		mp := Point3{X: mx, Y: my, Z: 0}

		var m1v, m2v Vector3
		P3Sub(&m1v, &vertices[list[k]], &mp)
		P3Sub(&m2v, &vertices[list[k+1]], &mp)

		var m1v2, m2v2 Vector3
		m1v2 = Vector3{
			X: m1v.X * rate,
			Y: m1v.Y * rate,
			Z: m1v.Z * rate,
		}

		m2v2 = Vector3{
			X: m2v.X * rate,
			Y: m2v.Y * rate,
			Z: m2v.Z * rate,
		}

		var n1, n2 Point3
		P3AddV3(&n1, &mp, &m1v2)
		P3AddV3(&n2, &mp, &m2v2)

		// fmt.Printf("old %v %v\n", vertices[list[k]], vertices[list[k+1]])
		// fmt.Printf("mid %v %v\n", mp, mp)
		// fmt.Printf("vec %v %v\n", m1v, m2v)
		// fmt.Printf("v2c %v %v\n", m1v2, m2v2)
		// fmt.Printf("new %v %v\n\n", n1, n2)

		nBorder[k] = int32(k)
		nBorder[k+1] = int32(k + 1)
		nVertices[k] = n1
		nVertices[k+1] = n2
	}
	return
}

func (nm *NavMesh) update_vis(v0 *Point3, vertices []Point3, indices []int32, i1, i2 int) (l, r *Vector3, left, right int) {
	var left_vec, right_vec, res Vector3
	P3Sub(&left_vec, &vertices[indices[i1]], v0)
	P3Sub(&right_vec, &vertices[indices[i2]], v0)
	V3Cross(&res, &left_vec, &right_vec)
	if res.Z > 0 {
		return &right_vec, &left_vec, i2, i1
	} else {
		return &left_vec, &right_vec, i1, i2
	}
}

func (nm *NavMesh) RouteWithRandOffset(list TriangleList,
	start, end *Point3,
	min, max float32) (*Path, error) {

	if min > max {
		max, min = min, max
	}
	r := Path{}
	// 计算临边
	border := nm.create_border(list.Triangles)
	// 将临边进行一定收缩变换.
	// vertices := list.Vertices
	border, vertices := nm.shrink_border(border, list.Vertices, 0.6+0.4*rand.Float32())

	// 目标点
	vertices = append(vertices, *end)
	border = append(border, int32(len(vertices))-1, int32(len(vertices))-1)

	// 第一个可视区域
	line_start := *start
	last_vis_left, last_vis_right, last_p_left, last_p_right := nm.update_vis(start, vertices, border, 0, 1)
	var res Vector3
	for k := 2; k <= len(border)-2; k += 2 {
		cur_vis_left, cur_vis_right, p_left, p_right := nm.update_vis(
			&line_start, vertices, border, k, k+1)
		V3Cross(&res, last_vis_left, cur_vis_right)
		if res.Z > 0 { // 左拐点
			line_start = vertices[border[last_p_left]]
			r.Line = append(r.Line, line_start)
			// 找到一条不共点的边作为可视区域
			i := 2 * (last_p_left/2 + 1)
			for ; i <= len(border)-2; i += 2 {
				if border[last_p_left] != border[i] && border[last_p_left] != border[i+1] {
					last_vis_left, last_vis_right, last_p_left, last_p_right = nm.update_vis(&line_start, vertices, border, i, i+1)
					break
				}
			}

			k = i
			continue
		}

		V3Cross(&res, last_vis_right, cur_vis_left)
		if res.Z < 0 { // 右拐点
			line_start = vertices[border[last_p_right]]
			r.Line = append(r.Line, line_start)
			// 找到一条不共点的边
			i := 2 * (last_p_right/2 + 1)
			for ; i <= len(border)-2; i += 2 {
				if border[last_p_right] != border[i] && border[last_p_right] != border[i+1] {
					last_vis_left, last_vis_right, last_p_left, last_p_right = nm.update_vis(&line_start, vertices, border, i, i+1)
					break
				}
			}

			k = i
			continue
		}

		V3Cross(&res, last_vis_left, cur_vis_left)
		if res.Z < 0 {
			last_vis_left = cur_vis_left
			last_p_left = p_left
		}

		V3Cross(&res, last_vis_right, cur_vis_right)
		if res.Z > 0 {
			last_vis_right = cur_vis_right
			last_p_right = p_right
		}
	}

	return &r, nil
}
