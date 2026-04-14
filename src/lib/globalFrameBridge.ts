/**
 * Global Frame Solver Bridge
 * ─────────────────────────────────────────────────────────────────
 * Converts the app's Beam/Column/Frame model into GFS types,
 * runs `solveGlobalFrame`, and maps results back to FrameResult[].
 *
 * This bridges the gap so the Global Frame engine and Unified Core
 * engine produce independent results instead of falling through
 * to the legacy 3D solver.
 */

import type { Beam, Column, Frame, FrameResult, MatProps, BeamOnBeamConnection, Slab, SlabProps } from '@/lib/structuralEngine';
import { calculateBeamLoads } from '@/lib/structuralEngine';
import {
  GlobalNodeRegistry,
  rectangularSection,
  solveGlobalFrame,
  type GFSElement,
  type GFSMaterial,
  type GFSLoad,
  type GFSNode,
  type GFSElementResult,
} from '@/lib/globalFrameSolver';

type EndReleaseMap = Record<string, {
  nodeI: { ux: boolean; uy: boolean; uz: boolean; rx: boolean; ry: boolean; rz: boolean };
  nodeJ: { ux: boolean; uy: boolean; uz: boolean; rx: boolean; ry: boolean; rz: boolean };
}>;

/**
 * Run the Global Frame Solver on the app model and return FrameResult[].
 */
export function getFrameResultsGlobalFrame(
  frames: Frame[],
  beams: Beam[],
  columns: Column[],
  mat: MatProps,
  frameEndReleases?: EndReleaseMap,
  beamOnBeamConnections?: BeamOnBeamConnection[],
  slabs?: Slab[],
  slabProps?: SlabProps,
  beamStiffnessFactor = 0.35,
  colStiffnessFactor = 0.65,
): FrameResult[] {
  const beamsMap = new Map(beams.map(b => [b.id, b]));
  const E_MPa = 4700 * Math.sqrt(mat.fc) * 1000; // N/mm²
  const G_MPa = E_MPa / (2 * (1 + 0.2));
  const gfsMat: GFSMaterial = { E: E_MPa, G: G_MPa };

  const registry = new GlobalNodeRegistry(1.0); // 1mm tolerance
  const elements: GFSElement[] = [];
  const elementLoads = new Map<string, { wx: number; wy: number; wz: number }>();

  // Track which GFS element IDs correspond to which beam IDs
  const beamElemIdMap = new Map<string, string>(); // beamId → gfsElemId

  // ── Determine ground level ──────────────────────────────────────────
  let minZ = Infinity;
  for (const col of columns) {
    if (col.isRemoved) continue;
    const zBot = col.zBottom ?? 0;
    if (zBot < minZ) minZ = zBot;
  }

  // ── Build column elements ───────────────────────────────────────────
  const colTopNodeMap = new Map<string, string>(); // colId → GFS nodeId

  for (const col of columns) {
    if (col.isRemoved) continue;
    const zBot = col.zBottom ?? 0;
    const zTop = col.zTop ?? (zBot + col.L);
    const xMm = col.x * 1000;
    const yMm = col.y * 1000;

    const isGroundLevel = Math.abs(zBot - minZ) < 1;
    const isPinned = col.bottomEndCondition === 'P';
    const botRestraints: GFSNode['restraints'] = isGroundLevel
      ? (isPinned
        ? [true, true, true, false, false, false]
        : [true, true, true, true, true, true])
      : [false, false, false, false, false, false];

    const botNode = registry.getOrCreateNode(xMm, yMm, zBot, botRestraints);
    const topNode = registry.getOrCreateNode(xMm, yMm, zTop, [false, false, false, false, false, false]);

    colTopNodeMap.set(col.id, topNode.id);

    const sec = rectangularSection(col.b, col.h);
    const elemId = `col_${col.id}`;
    elements.push({
      id: elemId,
      nodeI: botNode.id,
      nodeJ: topNode.id,
      section: sec,
      material: gfsMat,
      stiffnessModifier: colStiffnessFactor,
      type: 'column',
    });

    // Column self-weight as distributed load (global Z, negative = downward)
    const colSW = -1.2 * mat.gamma * (col.b * col.h) / 1e6; // kN/m
    elementLoads.set(elemId, { wx: 0, wy: 0, wz: colSW });
  }

  // ── Build beam elements ─────────────────────────────────────────────
  const processedBeams = new Set<string>();

  for (const frame of frames) {
    for (const beamId of frame.beamIds) {
      if (processedBeams.has(beamId)) continue;
      processedBeams.add(beamId);

      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      const fromCol = columns.find(c => c.id === beam.fromCol);
      const toCol = columns.find(c => c.id === beam.toCol);
      if (!fromCol || !toCol) continue;

      let nodeIId = colTopNodeMap.get(fromCol.id);
      let nodeJId = colTopNodeMap.get(toCol.id);

      const isBoBSecondary = beamOnBeamConnections?.some(
        c => c.secondaryBeamIds.includes(beamId)
      );

      // Create pinned supports for removed columns (non-BoB)
      if (!nodeIId && !isBoBSecondary) {
        const xMm = fromCol.x * 1000;
        const yMm = fromCol.y * 1000;
        const zTop = fromCol.zTop ?? ((fromCol.zBottom ?? 0) + fromCol.L);
        const node = registry.getOrCreateNode(xMm, yMm, zTop, [true, true, true, false, false, false]);
        nodeIId = node.id;
      }
      if (!nodeJId && !isBoBSecondary) {
        const xMm = toCol.x * 1000;
        const yMm = toCol.y * 1000;
        const zTop = toCol.zTop ?? ((toCol.zBottom ?? 0) + toCol.L);
        const node = registry.getOrCreateNode(xMm, yMm, zTop, [true, true, true, false, false, false]);
        nodeJId = node.id;
      }

      if (!nodeIId || !nodeJId) continue;

      const sec = rectangularSection(beam.b, beam.h);
      const elemId = `beam_${beamId}`;

      // End releases
      let releasesI: GFSElement['releasesI'];
      let releasesJ: GFSElement['releasesJ'];
      if (frameEndReleases) {
        const posKey = `${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}_${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}`;
        const posKeyRev = `${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}_${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}`;
        const rel = frameEndReleases[posKey] || frameEndReleases[posKeyRev];
        if (rel) {
          const isReversed = !!frameEndReleases[posKeyRev] && !frameEndReleases[posKey];
          const ni = isReversed ? rel.nodeJ : rel.nodeI;
          const nj = isReversed ? rel.nodeI : rel.nodeJ;
          releasesI = { Ux: ni.ux, Uy: ni.uy, Uz: ni.uz, Rx: ni.rx, Ry: ni.ry, Rz: ni.rz };
          releasesJ = { Ux: nj.ux, Uy: nj.uy, Uz: nj.uz, Rx: nj.rx, Ry: nj.ry, Rz: nj.rz };
        }
      }

      elements.push({
        id: elemId,
        nodeI: nodeIId,
        nodeJ: nodeJId,
        section: sec,
        material: gfsMat,
        stiffnessModifier: beamStiffnessFactor,
        type: 'beam',
        releasesI,
        releasesJ,
      });

      // Beam loads: factored UDL (gravity, global Z negative)
      const beamSW = (beam.b / 1000) * (beam.h / 1000) * mat.gamma;
      const wallLoad = beam.wallLoad ?? 0;
      const slabDL = beam.deadLoad ? (beam.deadLoad - beamSW - wallLoad) : 0;
      const totalDead = beamSW + wallLoad + Math.max(slabDL, 0);
      const totalLive = beam.liveLoad ?? 0;
      const wu = -(1.2 * totalDead + 1.6 * totalLive); // kN/m, negative Z = downward
      elementLoads.set(elemId, { wx: 0, wy: 0, wz: wu });

      beamElemIdMap.set(beamId, elemId);
    }
  }

  // ── Handle Beam-on-Beam connections ─────────────────────────────────
  if (beamOnBeamConnections) {
    for (const conn of beamOnBeamConnections) {
      for (const secBeamId of conn.secondaryBeamIds) {
        if (processedBeams.has(secBeamId)) continue;
        processedBeams.add(secBeamId);

        const beam = beamsMap.get(secBeamId);
        if (!beam) continue;

        const fromCol = columns.find(c => c.id === beam.fromCol);
        const toCol = columns.find(c => c.id === beam.toCol);
        if (!fromCol || !toCol) continue;

        // One end connects to primary beam intersection point
        const xFrom = fromCol.x * 1000, yFrom = fromCol.y * 1000;
        const xTo = toCol.x * 1000, yTo = toCol.y * 1000;
        const zFrom = fromCol.zTop ?? ((fromCol.zBottom ?? 0) + fromCol.L);
        const zTo = toCol.zTop ?? ((toCol.zBottom ?? 0) + toCol.L);

        const nodeI = registry.getOrCreateNode(xFrom, yFrom, zFrom,
          fromCol.isRemoved ? [false, false, false, false, false, false] : [false, false, false, false, false, false]);
        const nodeJ = registry.getOrCreateNode(xTo, yTo, zTo,
          toCol.isRemoved ? [false, false, false, false, false, false] : [false, false, false, false, false, false]);

        const sec = rectangularSection(beam.b, beam.h);
        const elemId = `beam_${secBeamId}`;

        // Secondary beam has moment release at removed column end
        let relI: GFSElement['releasesI'];
        let relJ: GFSElement['releasesJ'];
        if (fromCol.isRemoved) {
          relI = { Rz: true, Ry: true };
        }
        if (toCol.isRemoved) {
          relJ = { Rz: true, Ry: true };
        }

        elements.push({
          id: elemId,
          nodeI: nodeI.id,
          nodeJ: nodeJ.id,
          section: sec,
          material: gfsMat,
          stiffnessModifier: beamStiffnessFactor,
          type: 'beam',
          releasesI: relI,
          releasesJ: relJ,
        });

        const beamSW = (beam.b / 1000) * (beam.h / 1000) * mat.gamma;
        const wallLoad = beam.wallLoad ?? 0;
        const slabDL = beam.deadLoad ? (beam.deadLoad - beamSW - wallLoad) : 0;
        const totalDead = beamSW + wallLoad + Math.max(slabDL, 0);
        const totalLive = beam.liveLoad ?? 0;
        const wu = -(1.2 * totalDead + 1.6 * totalLive);
        elementLoads.set(elemId, { wx: 0, wy: 0, wz: wu });

        beamElemIdMap.set(secBeamId, elemId);
      }
    }
  }

  // ── Solve ───────────────────────────────────────────────────────────
  const nodes = registry.getAllNodes();
  const load: GFSLoad = { elementLoads };

  const result = solveGlobalFrame(nodes, elements, load);

  // ── Map results to FrameResult[] ────────────────────────────────────
  const elemResultMap = new Map<string, GFSElementResult>();
  for (const er of result.elementResults) {
    elemResultMap.set(er.elementId, er);
  }

  // Build beam release lookup for zeroing released-end moments
  const beamReleaseLookup = new Map<string, { relI_mz: boolean; relJ_mz: boolean }>();
  if (frameEndReleases) {
    for (const beam of beams) {
      const fromCol = columns.find(c => c.id === beam.fromCol);
      const toCol = columns.find(c => c.id === beam.toCol);
      if (!fromCol || !toCol) continue;
      const posKey = `${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}_${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}`;
      const posKeyRev = `${toCol.x.toFixed(3)}_${toCol.y.toFixed(3)}_${fromCol.x.toFixed(3)}_${fromCol.y.toFixed(3)}`;
      const rel = frameEndReleases[posKey] || frameEndReleases[posKeyRev];
      if (!rel) continue;
      const isReversed = !!frameEndReleases[posKeyRev] && !frameEndReleases[posKey];
      const ni = isReversed ? rel.nodeJ : rel.nodeI;
      const nj = isReversed ? rel.nodeI : rel.nodeJ;
      beamReleaseLookup.set(beam.id, { relI_mz: ni.rz, relJ_mz: nj.rz });
    }
  }

  return frames.map((frame): FrameResult => {
    const frameBeams: FrameResult['beams'] = [];

    for (const beamId of frame.beamIds) {
      const beam = beamsMap.get(beamId);
      if (!beam) continue;

      const elemId = beamElemIdMap.get(beamId);
      const er = elemId ? elemResultMap.get(elemId) : undefined;

      if (!er) {
        frameBeams.push({
          beamId, span: beam.length,
          Mleft: 0, Mmid: 0, Mright: 0, Vu: 0, Rleft: 0, Rright: 0,
        });
        continue;
      }

      // GFS momentZI/momentZJ are already in structural sign convention
      // momentZI: hogging < 0, momentZJ: hogging < 0
      let Mleft = er.momentZI;
      let Mright = er.momentZJ;

      // Enforce zero at released ends
      const rel = beamReleaseLookup.get(beamId);
      if (rel) {
        if (rel.relI_mz) Mleft = 0;
        if (rel.relJ_mz) Mright = 0;
      }

      frameBeams.push({
        beamId,
        span: beam.length,
        Mleft,
        Mmid: er.momentZmid,
        Mright,
        Vu: er.shearY,
        Rleft: Math.abs(er.forceI[2]),  // Fz at node I (kN)
        Rright: Math.abs(er.forceJ[2]), // Fz at node J (kN)
      });
    }

    return { frameId: frame.id, beams: frameBeams };
  });
}
