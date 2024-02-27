import {
  JupyterFrontEnd,
  JupyterFrontEndPlugin
} from '@jupyterlab/application';

import { 
  IBlocklyRegistry,
  THEME,
} from 'jupyterlab-blockly';

THEME.fontStyle = {
  weight: 'bold'
}

import BlocklyGiskard from './giskard_blocks_and_toolbox';
import BlocklyIpylgbst from './ipylgbst_blocks_and_toolbox';

/**
 * Initialization data for the jupyterlab-blockly-ipylgbst extension.
 */
const plugin: JupyterFrontEndPlugin<void> = {
  id: 'jupyterlab-blockly-ipylgbst:plugin',
  autoStart: true,
  requires: [IBlocklyRegistry],
  activate: (app: JupyterFrontEnd, blockly: IBlocklyRegistry) => {
    console.log(
      'JupyterLab extension jupyterlab-blockly-ipylgbst is activated!'
    );

    blockly.registerToolbox('default', BlocklyGiskard.Toolbox);
    // blockly.registerToolbox('giskard', BlocklyGiskard.Toolbox);
    blockly.registerToolbox('ipylgbst', BlocklyIpylgbst.Toolbox);
  }
};

export default plugin;
