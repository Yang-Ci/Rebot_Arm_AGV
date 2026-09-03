const MAX_VISIBLE_CONTACTS = 40;

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

export function createContactTelemetry(mujoco, model, data) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const fingerBodies = new Set([
    namedId(mujoco, model, bodyType, 'gripper_left'),
    namedId(mujoco, model, bodyType, 'gripper_right')
  ]);
  const objectBodies = {
    red: namedId(mujoco, model, bodyType, 'red_cube'),
    blue: namedId(mujoco, model, bodyType, 'blue_block'),
    yellow: namedId(mujoco, model, bodyType, 'yellow_cylinder')
  };
  const force = new Float64Array(6);

  function sample(selectedId = 'red', includeVisuals = true) {
    const visible = [];
    const selectedBody = objectBodies[selectedId] ?? -1;
    let graspForce = 0;
    let activeContacts = 0;
    let peakForce = 0;
    const count = Math.min(data.ncon, data.contact.size());

    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      force.fill(0);
      mujoco.mj_contactForce(model, data, index, force);
      const normalForce = Math.abs(force[0]);
      if (!Number.isFinite(normalForce) || normalForce < 0.01) continue;
      activeContacts += 1;
      peakForce = Math.max(peakForce, normalForce);

      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      const selectedGrip =
        (fingerBodies.has(body1) && body2 === selectedBody) ||
        (fingerBodies.has(body2) && body1 === selectedBody);
      if (selectedGrip) graspForce += normalForce;

      if (includeVisuals && visible.length < MAX_VISIBLE_CONTACTS) {
        const direction = body2 === selectedBody ? 1 : -1;
        visible.push({
          position: [contact.pos[0], contact.pos[1], contact.pos[2]],
          normal: [
            contact.frame[0] * direction,
            contact.frame[1] * direction,
            contact.frame[2] * direction
          ],
          force: normalForce,
          selectedGrip
        });
      }
    }

    return { contacts: visible, activeContacts, graspForce, peakForce };
  }

  return { sample };
}
