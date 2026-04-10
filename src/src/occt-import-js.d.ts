declare module 'occt-import-js' {
  interface OcctMesh {
    attributes: {
      position: { array: number[] }
      normal?: { array: number[] }
    }
    index?: { array: number[] }
    color?: [number, number, number]
  }

  interface OcctNode {
    name?: string
    children?: OcctNode[]
    meshes?: number[]
  }

  interface OcctResult {
    success: boolean
    meshes: OcctMesh[]
    root?: OcctNode
  }

  interface OcctModule {
    ReadStepFile(buffer: Uint8Array, params: null): OcctResult
    ReadIgesFile(buffer: Uint8Array, params: null): OcctResult
    ReadBrepFile(buffer: Uint8Array, params: null): OcctResult
  }

  export default function occtimportjs(options?: { locateFile?: (filename: string) => string }): Promise<OcctModule>
}
