import * as Blockly from 'blockly';
import { pythonGenerator as BlocklyPy } from 'blockly/python';

import BlocklyBase from './basic_blocks_toolbox';

const giskard_color = '#000000';
var ipylgbst_color_async = '#5D3FD3';
var vernie_color_async = '#088F8F';
var ipylgbst_color = '#8169df';
var vernie_color = '#40B5AD';


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
      this.appendDummyInput()
      this.setPreviousStatement(true, null)
      this.setNextStatement(true, null)
      this.setInputsInline(true)
      this.setColour(giskard_color)
      this.setTooltip('Start a robot simulator.')
      this.setHelpUrl('')
    },
    // The python code generator for the block
    generator: (block) => {
      var robot = BlocklyPy.valueToCode(block, 'ROBOT', BlocklyPy.ORDER_ATOMIC);
      var code = [
        `start_robot(${robot})`,
      ].join('\n');
      return code + '\n';
    },
    // The python top-level code, for example import libraries
    toplevel_init: [
      `from utils import start_robot`,
      `import rospy`,
    ].join('\n') + '\n'
  },
  {
    id: 'giskard_sleep',
    block_init: function() {
      this.appendValueInput('TIME').setCheck('Number').appendField('Wait for');
      this.appendDummyInput().appendField('seconds');
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setInputsInline(true);
      this.setColour(ipylgbst_color_async);
      this.setTooltip('Wait for a certain amount of time.');
      this.setHelpUrl('');
    },
    generator: (block) => {
      var value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      var code = `rospy.sleep(${value_time})\n`;
      return code;
    }
  },
  {
    id: 'giskard_moverobot',
    block_init: function() {
      this.appendValueInput('SPEED')
        .setCheck('Number')
        .appendField('Move forward with speed');
      this.appendValueInput('TIME').setCheck('Number').appendField('for');
      this.appendDummyInput().appendField('seconds');
      this.setInputsInline(true);
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(ipylgbst_color_async);
      this.setTooltip(
        'Move forward with a certain speed for a chosen no. of seconds.'
      );
      this.setHelpUrl('');
    },
    generator: (block) => {
      var value_speed = BlocklyPy.valueToCode(block, 'SPEED', BlocklyPy.ORDER_ATOMIC);
      var value_time = BlocklyPy.valueToCode(block, 'TIME', BlocklyPy.ORDER_ATOMIC);
      var code = `move_forward(${value_speed}, ${value_time})\n`;
      return code;
    },
    toplevel_init: `from utils import move_forward\n`
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
      colour: giskard_color,
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
