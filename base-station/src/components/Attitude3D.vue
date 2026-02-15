<template>
    <div ref="root" class="attitude"></div>
</template>
<script setup lang="ts">
import { onMounted, onBeforeUnmount, ref, watch } from "vue"
import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls"

import { useTelemetryStore } from "../stores/telemetry";

const store = useTelemetryStore()
const root = ref<HTMLDivElement | null>(null)

let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene | null = null
let camera: THREE.PerspectiveCamera | null = null
let controls: OrbitControls | null = null
let raf = 0
let ro: ResizeObserver | null = null

let drone : THREE.Object3D | null = null

const targetQ = new THREE.Quaternion()

onMounted(()=> {
    const el = root.value!
    scene = new THREE.Scene()
    scene.background = new THREE.Color(0xffffff)

    const w = el.clientWidth
    const h = 300

    camera = new THREE.PerspectiveCamera(50, w/h, 0.01, 100)
    camera.position.set(1.5,1.0,1.5)
    camera.lookAt(0,0,0)

    renderer = new THREE.WebGLRenderer({antialias:true})
    renderer.setSize(w,h)
    el.appendChild(renderer.domElement)

    scene.add(new THREE.AxesHelper(0.5))
    scene.add(new THREE.GridHelper(4, 20))

    scene.add(new THREE.HemisphereLight(0xffffff,0x444444,1.0))
    const dir = new THREE.DirectionalLight(0xffffff,0.8)
    dir.position.set(2,3,2)
    scene.add(dir)

    const body = new THREE.Mesh(
        new THREE.BoxGeometry(0.4,0.08,0.25),
        new THREE.MeshStandardMaterial({color:0xdddddd})
    )

    const nose = new THREE.Mesh(
        new THREE.ConeGeometry(0.06,0.18,16),
        new THREE.MeshStandardMaterial({color:0xff6666})
    )
    nose.rotation.x = Math.PI / 2
    nose.position.set(0,0,-0.22)

    drone = new THREE.Group()
    drone.add(body,nose)
    scene.add(drone)

    controls = new OrbitControls(camera,renderer.domElement)
    controls.enableDamping = true

    ro = new ResizeObserver(()=> {
        if (!renderer  || !camera || !root.value) return
        const nw = root.value.clientWidth
        const nh = 300
        renderer.setSize(nw,nh)
        camera.aspect = nw / nh
        camera.updateProjectionMatrix()
    })
    ro.observe(el)



    const loop = () => {
        raf = requestAnimationFrame(loop)
        if (!renderer || !scene || !camera || !drone) return

        drone.quaternion.slerp(targetQ,0.25)
        controls?.update()
        renderer.render(scene,camera)
    }
    loop()
})

onBeforeUnmount(()=>{
    cancelAnimationFrame(raf)
    ro?.disconnect()
    controls?.dispose()
    renderer?.dispose()
    renderer?.domElement?.remove()

    renderer =null
    scene = null
    camera=null
    controls=null
    drone=null
})

watch(
  () => store.attitudeEuler,
  (a) => {
    if (!a) return

    const toRad = (v: number) =>
      (a.unit ?? "rad") === "deg" ? THREE.MathUtils.degToRad(v) : v

    const roll  = toRad(a.roll)
    const pitch = toRad(a.pitch)
    const yaw   = toRad(a.yaw)

    // Model forward = -Z (your nose is at z=-0.22)
    // Map aircraft: roll about forward(-Z), pitch about right(+X), yaw about up(+Y)
    // => Three: x=pitch, y=yaw, z=-roll
    const e = new THREE.Euler(roll, yaw, -pitch, "YXZ") // yaw->pitch->roll
    targetQ.setFromEuler(e).normalize()
  }
)

</script>

<style lang="css" scoped>
.attitude{
    width: 100%
}
</style>