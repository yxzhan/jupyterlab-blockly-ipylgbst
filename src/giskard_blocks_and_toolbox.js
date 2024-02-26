import * as Blockly from 'blockly';
import { pythonGenerator as BlocklyPy } from 'blockly/python';

import BlocklyBase from './basic_blocks_toolbox';

const giskard_colors = [
    '#418F55',
    '#3F4545',
    '#AD7339',
    '#FFA42E',
    '#FF4F4F'
]

/*
 * Block definitions
 */

const CUSTOM_BLOCKS = [
  {
    // unique id of a type of block
    id: 'giskard_start',
    // Function defines how the block will look like, the inputs, the fields, etc.
    // Docs: https://developers.google.com/blockly/guides/create-custom-blocks/define-blocks#javascript_2
    block_init: function() {
      this.appendValueInput('ROBOT')
        .setCheck('String')
        .appendField('Start robot')
      this.setPreviousStatement(false, null)
      this.setNextStatement(true, null)
      this.setInputsInline(true)
      this.setColour(giskard_colors[0])
      this.setTooltip('Start a robot simulator.')
      this.setHelpUrl('')
    },
    // The python code generator for the block
    generator: (block) => {
      let robot = BlocklyPy.valueToCode(block, 'ROBOT', BlocklyPy.ORDER_ATOMIC);
      let code = `blockly_start(${robot})\n`
      return code;
    },
    // The python top-level code, for example import libraries
    toplevel_init: [
      `import rospy`,
      `from utils import blockly_start, blockly_move, blockly_turn`,
    ].join('\n') + '\n'
  },
  {
    id: 'giskard_sleep',
    block_init: function() {
      this.appendValueInput('TIME')
        .setCheck('Number')
        .appendField('Wait for');
      this.appendDummyInput()
        .appendField('seconds');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setInputsInline(true);
      this.setColour(giskard_colors[1])
      this.setTooltip('Wait for a certain amount of time.');
      this.setHelpUrl('');
    },
    generator: (block) => {
      let value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      let code = `rospy.sleep(${value_time})\n`;
      return code;
    }
  },
  {
    id: 'giskard_forward',
    block_init: function() {
      this.appendValueInput('SPEED')
        .setCheck('Number')
        .appendField('Move forward with speed');
      this.appendValueInput('TIME')
        .setCheck('Number')
        .appendField('for');
      this.appendDummyInput()
        .appendField('seconds');
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(giskard_colors[3])
      this.setTooltip(
        'Move forward with a certain speed for a chosen no. of seconds.'
      );
      this.setHelpUrl('');
    },
    generator: (block) => {
      let value_speed = BlocklyPy.valueToCode(block, 'SPEED', BlocklyPy.ORDER_ATOMIC);
      let value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      let code = `blockly_move(${value_speed}, ${value_time})\n`;
      return code;
    }
  },
  {
    id: 'giskard_backward',
    block_init: function() {
      this.appendValueInput('SPEED')
        .setCheck('Number')
        .appendField('Move backward with speed');
      this.appendValueInput('TIME')
        .setCheck('Number')
        .appendField('for');
      this.appendDummyInput()
        .appendField('seconds');
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(giskard_colors[3])
      this.setTooltip(
        'Move backward with a certain speed for a chosen no. of seconds.'
      );
      this.setHelpUrl('');
    },
    generator: (block) => {
      let value_speed = BlocklyPy.valueToCode(block, 'SPEED', BlocklyPy.ORDER_ATOMIC);
      let value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      let code = `blockly_move(-${value_speed}, ${value_time})\n`;
      return code;
    }
  },
  {
    id: 'giskard_turn_left',
    block_init: function() {
      this.appendValueInput('SPEED')
        .setCheck('Number')
        .appendField('Turn left with speed');
      this.appendValueInput('TIME')
        .setCheck('Number')
        .appendField('for');
      this.appendDummyInput().appendField('seconds');
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(giskard_colors[4])
      this.setTooltip(
        'Turn left with a certain speed for a chosen no. of seconds.'
      );
      this.setHelpUrl('');
    },
    generator: (block) => {
      let value_speed = BlocklyPy.valueToCode(block, 'SPEED', BlocklyPy.ORDER_ATOMIC);
      let value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      let code = `blockly_turn(${value_speed}, ${value_time})\n`;
      return code;
    }
  },
  {
    id: 'giskard_turn_right',
    block_init: function() {
      this.appendValueInput('SPEED')
        .setCheck('Number')
        .appendField('Turn right with speed');
      this.appendValueInput('TIME')
        .setCheck('Number')
        .appendField('for');
      this.appendDummyInput().appendField('seconds');
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(giskard_colors[4])
      this.setTooltip(
        'Turn right with a certain speed for a chosen no. of seconds.'
      );
      this.setHelpUrl('');
    },
    generator: (block) => {
      var value_speed = BlocklyPy.valueToCode(block, 'SPEED', BlocklyPy.ORDER_ATOMIC);
      var value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      var code = `blockly_turn(-${value_speed}, ${value_time})\n`;
      return code;
    }
  }
]

// Registering the custom blocks and their generators in blockly
for (const blocks of CUSTOM_BLOCKS) {
  Blockly.Blocks[blocks.id] = {
    init: blocks.block_init
  };

  BlocklyPy[blocks.id] = blocks.generator;
  if (blocks.toplevel_init) {
    Blockly.Blocks[blocks.id].toplevel_init = blocks.toplevel_init;
  }
}

// Creating a toolbox extends the base blocks toolbox by adding the custom blocks
const TOOLBOX = {
  kind: 'categoryToolbox',
  contents: [
    ...BlocklyBase.Toolbox.contents,
    {
      kind: 'CATEGORY',
      colour: giskard_colors[0],
      name: 'Robotics',
      contents: CUSTOM_BLOCKS.map(v => {
        return {
          kind: 'BLOCK',
          type: v.id
        }
      })
    }
  ]
};

const BlocklyGiskard = {
  Blocks: Blockly.Blocks,
  Generator: BlocklyPy,
  Toolbox: TOOLBOX
};

export default BlocklyGiskard;
