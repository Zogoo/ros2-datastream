import { describe, expect, it } from 'vitest';
import { bumperSector } from '../src/sensors/contacts.js';

describe('bumper ring sectors (360°)', () => {
  it('maps bearings to the 8 skirt sectors', () => {
    expect(bumperSector(0)).toBe('front');
    expect(bumperSector(20)).toBe('front');
    expect(bumperSector(45)).toBe('front_left');
    expect(bumperSector(90)).toBe('left');
    expect(bumperSector(135)).toBe('rear_left');
    expect(bumperSector(180)).toBe('rear');
    expect(bumperSector(-180)).toBe('rear');
    expect(bumperSector(-135)).toBe('rear_right');
    expect(bumperSector(-90)).toBe('right');
    expect(bumperSector(-45)).toBe('front_right');
    expect(bumperSector(-20)).toBe('front');
  });

  it('unresolvable force defaults to front (conservative escape)', () => {
    expect(bumperSector(null)).toBe('front');
    expect(bumperSector(NaN)).toBe('front');
  });
});
