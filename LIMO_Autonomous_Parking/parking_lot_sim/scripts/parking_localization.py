"""Scan likelihood, discrete Markov localization and Monte Carlo tracking."""

import math

import numpy as np

from parking_core import inverse, scan_rays
from patrol_route import Point, transform, wrap


def localization_hits(scan):
    """Subsample valid endpoints, retaining evidence in sparsely covered scans."""
    hits = np.asarray([(x, y) for x, y, hit in scan_rays(scan, Point(0, 0, 0, 0), 26.0) if hit])
    if len(hits) > 64:
        hits = hits[np.linspace(0, len(hits)-1, 64, dtype=int)]
    return hits.reshape(-1, 2)


class ScanLikelihood:
    def __init__(self, data, resolution, origin):
        self.origin = inverse(origin)
        self.resolution = resolution
        self.data = data
        occupied = data >= 65
        self.distances = np.full(data.shape, 0.9, dtype=np.float32)
        rows, cols = data.shape
        radius = math.ceil(0.9/resolution)
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                distance = math.hypot(dx, dy)*resolution
                if distance >= 0.9 or abs(dy) >= rows or abs(dx) >= cols:
                    continue
                source = occupied[max(0, -dy):min(rows, rows-dy), max(0, -dx):min(cols, cols-dx)]
                target = self.distances[max(0, dy):min(rows, rows+dy), max(0, dx):min(cols, cols+dx)]
                np.minimum(target, distance, out=target, where=source)

    def cells(self, x, y):
        ox, oy, angle = self.origin
        c, s = math.cos(angle), math.sin(angle)
        ix = np.floor((c*x-s*y+ox)/self.resolution).astype(int)
        iy = np.floor((s*x+c*y+oy)/self.resolution).astype(int)
        rows, cols = self.data.shape
        inside = (ix >= 0) & (ix < cols) & (iy >= 0) & (iy < rows)
        return np.clip(iy, 0, rows-1), np.clip(ix, 0, cols-1), inside

    def score(self, poses, hits):
        x, y, yaw = poses.T[:, :, None]
        c, s = np.cos(yaw), np.sin(yaw)
        hx, hy = hits.T
        iy, ix, inside = self.cells(x+c*hx-s*hy, y+s*hx+c*hy)
        distances = np.where(inside, self.distances[iy, ix], 0.9)
        match = np.mean(distances <= 0.38, axis=1)
        # A uniform outlier term limits the effect of moving and unmapped objects.
        likelihood = np.log(0.15+0.85*np.exp(-0.5*(distances/0.22)**2)).mean(axis=1)
        iy, ix, inside = self.cells(poses[:, 0], poses[:, 1])
        likelihood -= np.where(inside & (self.data[iy, ix] >= 0) & (self.data[iy, ix] < 65), 0, 1.0)
        return likelihood, match


def posterior(prior, score, strength):
    weights = prior*np.exp(strength*(score-np.max(score)))
    return weights/weights.sum()


def propagate(states, delta):
    c, s = np.cos(states[:, 2]), np.sin(states[:, 2])
    return states+np.column_stack((c*delta[0]-s*delta[1], s*delta[0]+c*delta[1],
                                   np.full(len(states), delta[2])))


def estimate(states, weights, matches, position_radius, angular_radius):
    best = states[np.argmax(weights)]
    angles = np.arctan2(np.sin(states[:, 2]-best[2]), np.cos(states[:, 2]-best[2]))
    cluster = ((np.linalg.norm(states[:, :2]-best[:2], axis=1) <= position_radius)
               & (np.abs(angles) <= angular_radius))
    mass = weights[cluster].sum()
    x, y = np.average(states[cluster, :2], weights=weights[cluster], axis=0)
    yaw = wrap(best[2]+np.average(angles[cluster], weights=weights[cluster]))
    return (float(x), float(y), yaw), float(mass), float(np.average(matches[cluster], weights=weights[cluster]))


class MarkovGrid:
    """A bounded histogram over map x, y and heading, with a discrete transition."""

    def __init__(self, seed):
        self.step = np.array((0.4, 0.4, math.radians(5)))
        half = np.array((8, 8, 6))
        self.shape = tuple(2*half+1)
        self.indices = np.indices(self.shape).reshape(3, -1).T
        self.states = np.array(seed)+(self.indices-half)*self.step
        self.weights = np.full(len(self.states), 1/len(self.states))
        self.updates = 0

    def update(self, model, delta, hits):
        predicted = self.weights.reshape(self.shape)
        indexes = np.rint((propagate(self.states, delta)-self.states[0])/self.step).astype(int)
        indexes = np.clip(indexes, 0, np.array(self.shape)-1)
        moved = np.zeros(self.shape)
        np.add.at(moved, tuple(indexes.T), predicted.ravel())
        diffusion = moved*0.88
        for axis in range(3):
            moved_axis = np.moveaxis(moved, axis, 0)
            spread = np.moveaxis(diffusion, axis, 0)
            spread[1:] += 0.02*moved_axis[:-1]
            spread[:-1] += 0.02*moved_axis[1:]
            spread[0] += 0.02*moved_axis[0]
            spread[-1] += 0.02*moved_axis[-1]
        scores, matches = model.score(self.states, hits)
        self.weights = posterior(diffusion.ravel(), scores, 13)
        self.updates += 1
        return estimate(self.states, self.weights, matches, 0.85, math.radians(12))


class ParticleFilter:
    def __init__(self, grid, rng, count=320):
        self.rng = rng
        self.count = count
        self.weights = np.full(count, 1/count)
        self.states = grid.states[self.resample(grid.weights)].copy()
        self.states += rng.uniform(-0.5, 0.5, self.states.shape)*grid.step

    def resample(self, weights):
        cumulative = np.cumsum(weights)
        cumulative[-1] = 1.0
        return np.searchsorted(cumulative, (self.rng.random()+np.arange(self.count))/self.count)

    def update(self, model, delta, hits):
        distance = math.hypot(*delta[:2])
        self.states = propagate(self.states, delta)
        deviation = (0.025+0.07*distance, 0.018+0.05*distance,
                     0.004+0.018*distance+0.12*abs(delta[2]))
        self.states += self.rng.normal(0.0, deviation, self.states.shape)
        scores, matches = model.score(self.states, hits)
        self.weights = posterior(self.weights, scores, 8)
        result = estimate(self.states, self.weights, matches, 0.65, math.radians(9))
        if 1/np.dot(self.weights, self.weights) < 0.7*self.count:
            self.states = self.states[self.resample(self.weights)]
            self.weights.fill(1/self.count)
        return result


class CourseLocalization:
    """Use fresh scans once each, first for the histogram and then the particles."""

    def __init__(self, model, alignment, initial_offset=(0.0, 0.0, 0.0)):
        self.model = model
        self.alignment = None
        self.seed_alignment = (alignment[0]+initial_offset[0], alignment[1]+initial_offset[1],
                               alignment[2])
        self.heading_offset = initial_offset[2]
        self.rng = np.random.default_rng(12)
        self.grid = self.particles = self.previous = None
        self.bad = 0

    def update(self, scan_pose, hits):
        current = (scan_pose.x, scan_pose.y, scan_pose.yaw)
        if self.previous is None:
            guess = transform(scan_pose, self.seed_alignment)
            self.grid = MarkovGrid((guess.x, guess.y, guess.yaw+self.heading_offset))
            delta = (0.0, 0.0, 0.0)
        else:
            motion = transform(scan_pose, inverse(self.previous))
            delta = (motion.x, motion.y, motion.yaw)
        self.previous = current
        if self.particles is None:
            pose, mass, match = self.grid.update(self.model, delta, hits)
            if self.grid.updates >= 3 and mass >= 0.63 and match >= 0.36:
                self.particles = ParticleFilter(self.grid, self.rng)
                return "markov_initialized_particles", pose, mass, match
            return "markov_search", pose, mass, match
        pose, mass, match = self.particles.update(self.model, delta, hits)
        if mass < 0.60 or match < 0.34:
            self.bad += 1
            if self.bad >= 5:
                self.seed_alignment = self.alignment or self.seed_alignment
                self.heading_offset = 0.0
                self.alignment = self.particles = self.previous = None
                self.bad = 0
                return "particle_relocalizing", pose, mass, match
            return "particle_low_confidence", pose, mass, match
        self.bad = 0
        tx, ty, angle = inverse(current)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        measured = (pose[0]+c*tx-s*ty, pose[1]+s*tx+c*ty, wrap(pose[2]+angle))
        if self.alignment is None:
            self.alignment = measured
        else:
            expected = transform(scan_pose, self.alignment)
            heading = wrap(self.alignment[2]+0.30*wrap(measured[2]-self.alignment[2]))
            c, s = math.cos(heading), math.sin(heading)
            self.alignment = (expected.x+0.30*(pose[0]-expected.x)-c*current[0]+s*current[1],
                              expected.y+0.30*(pose[1]-expected.y)-s*current[0]-c*current[1], heading)
        return "particle_tracking", pose, mass, match
